#include "visualization/ui/TacticalOverlays.h"

#include "foundation/core/Units.h"
#include "visualization/scene/Frames.h"

#include <vsgImGui/imgui.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace fsim::ui {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNearClipViewZ = -1.5; // View-space Z is negative forward in VSG/Vulkan right-handed lookAt

vsg::dvec3 bodyToEcefVec(const sim::VehicleState& s, const vsg::dvec3& b) noexcept {
    const double* R = s.rotationBodyToEcef;
    return vsg::dvec3(
        R[0] * b.x + R[1] * b.y + R[2] * b.z,
        R[3] * b.x + R[4] * b.y + R[5] * b.z,
        R[6] * b.x + R[7] * b.y + R[8] * b.z);
}

void localEnu(const vsg::dvec3& pos, vsg::dvec3& east, vsg::dvec3& north, vsg::dvec3& up) noexcept {
    up = world::localUp(pos);
    east = vsg::cross(vsg::dvec3(0.0, 0.0, 1.0), up);
    if (vsg::length(east) < 1e-6) east = vsg::dvec3(1.0, 0.0, 0.0);
    east = vsg::normalize(east);
    north = vsg::cross(up, east);
}

void orthoBasis(const vsg::dvec3& dir, vsg::dvec3& u, vsg::dvec3& v) noexcept {
    const vsg::dvec3 ref = std::abs(dir.z) < 0.9 ? vsg::dvec3(0.0, 0.0, 1.0) : vsg::dvec3(1.0, 0.0, 0.0);
    u = vsg::normalize(vsg::cross(ref, dir));
    v = vsg::normalize(vsg::cross(dir, u));
}

ImU32 weatherDbzColor(float dbz, std::uint8_t alpha) noexcept {
    if (dbz >= 52.0f) return IM_COL32(220, 45, 220, alpha); // Extreme hail/convective core
    if (dbz >= 42.0f) return IM_COL32(245, 65, 45, alpha);  // Heavy convective storm
    if (dbz >= 32.0f) return IM_COL32(245, 210, 45, alpha); // Moderate-heavy rain
    if (dbz >= 20.0f) return IM_COL32(60, 215, 90, alpha);  // Light-moderate rain
    return IM_COL32(70, 185, 210, alpha);                   // Light stratiform
}

void drawTag(ImDrawList* dl, float x, float y, const char* text, ImU32 textCol, ImU32 bgCol, float scale) {
    if (!text || !text[0]) return;
    const ImVec2 sz = ImGui::CalcTextSize(text);
    const float padX = 4.0f * scale, padY = 2.0f * scale;
    const ImVec2 lo(x - padX, y - padY);
    const ImVec2 hi(x + sz.x + padX, y + sz.y + padY);
    dl->AddRectFilled(lo, hi, bgCol, 3.0f * scale);
    dl->AddText(ImVec2(x, y), textCol, text);
}

} // namespace

TacticalOverlays::TacticalOverlays(std::shared_ptr<ViewerControls> controls, std::uint32_t deviceID)
    : controls_(std::move(controls)), deviceID_(deviceID) {}

void TacticalOverlays::setPipSource(vsg::ref_ptr<vsg::DescriptorSet> descriptorSet, std::uint32_t width, std::uint32_t height) {
    pipDescriptorSet_ = std::move(descriptorSet);
    pipWidth_ = std::max(160u, width);
    pipHeight_ = std::max(90u, height);
}

void TacticalOverlays::update(const ipc::TacticalOverlayFrameV1* frame,
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
                              std::string pipSubtitle) {
    cameraMode_ = cameraMode;
    selectedSlot_ = selectedSlot;
    view_ = viewMatrix;
    proj_ = projMatrix;
    viewProj_ = projMatrix * viewMatrix;
    eye_ = eyeEcef;
    width_ = std::max(1u, screenWidth);
    height_ = std::max(1u, screenHeight);
    simTime_ = simTime;
    pipActive_ = pipActive;
    pipTitle_ = std::move(pipTitle);
    pipSubtitle_ = std::move(pipSubtitle);

    const std::uint32_t mask = controls_ ? controls_->tacticalOverlayMask.load(std::memory_order_relaxed) : 0u;
    const bool needData = (mask != 0u) || (cameraMode_ == world::CameraController::Mode::Cockpit);
    if (!needData) {
        hasFrame_ = false;
        states_.clear();
        alive_.clear();
        return;
    }

    if (frame) {
        frame_ = *frame;
        hasFrame_ = true;
    } else {
        frame_ = ipc::TacticalOverlayFrameV1{};
        hasFrame_ = false;
    }
    states_.assign(states.begin(), states.end());
    alive_ = alive;
    meta_ = meta;
}

bool TacticalOverlays::projectPoint(const vsg::dvec3& ecef, float& sx, float& sy) const noexcept {
    const vsg::dvec4 v = view_ * vsg::dvec4(ecef.x, ecef.y, ecef.z, 1.0);
    if (v.z > kNearClipViewZ) return false;
    const vsg::dvec4 clip = proj_ * v;
    if (clip.w <= 1e-9) return false;
    const double nx = clip.x / clip.w;
    const double ny = clip.y / clip.w;
    if (!std::isfinite(nx) || !std::isfinite(ny) || nx < -2.5 || nx > 2.5 || ny < -2.5 || ny > 2.5) return false;
    sx = static_cast<float>((nx * 0.5 + 0.5) * static_cast<double>(width_));
    sy = static_cast<float>((ny * 0.5 + 0.5) * static_cast<double>(height_));
    return true;
}

void TacticalOverlays::drawLine3D(void* drawList, const vsg::dvec3& a, const vsg::dvec3& b, std::uint32_t col, float thickness) const {
    auto* dl = static_cast<ImDrawList*>(drawList);
    vsg::dvec4 va = view_ * vsg::dvec4(a.x, a.y, a.z, 1.0);
    vsg::dvec4 vb = view_ * vsg::dvec4(b.x, b.y, b.z, 1.0);
    const bool aIn = va.z <= kNearClipViewZ;
    const bool bIn = vb.z <= kNearClipViewZ;
    if (!aIn && !bIn) return;
    if (!aIn) {
        const double t = (kNearClipViewZ - va.z) / (vb.z - va.z);
        va = va + (vb - va) * std::clamp(t, 0.0, 1.0);
    } else if (!bIn) {
        const double t = (kNearClipViewZ - vb.z) / (va.z - vb.z);
        vb = vb + (va - vb) * std::clamp(t, 0.0, 1.0);
    }
    const vsg::dvec4 ca = proj_ * va;
    const vsg::dvec4 cb = proj_ * vb;
    if (ca.w <= 1e-9 || cb.w <= 1e-9) return;
    const double ax = (ca.x / ca.w * 0.5 + 0.5) * static_cast<double>(width_);
    const double ay = (ca.y / ca.w * 0.5 + 0.5) * static_cast<double>(height_);
    const double bx = (cb.x / cb.w * 0.5 + 0.5) * static_cast<double>(width_);
    const double by = (cb.y / cb.w * 0.5 + 0.5) * static_cast<double>(height_);
    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(bx) || !std::isfinite(by)) return;
    const double pad = std::max<double>(width_, height_) * 1.5;
    if (std::min(ax, bx) > width_ + pad || std::max(ax, bx) < -pad ||
        std::min(ay, by) > height_ + pad || std::max(ay, by) < -pad) return;
    dl->AddLine(ImVec2(static_cast<float>(ax), static_cast<float>(ay)),
                ImVec2(static_cast<float>(bx), static_cast<float>(by)), col, thickness);
}

void TacticalOverlays::drawDashedLine3D(void* drawList, const vsg::dvec3& a, const vsg::dvec3& b, std::uint32_t col, float thickness, int segments) const {
    const int n = std::clamp(segments, 2, 32);
    for (int i = 0; i < n; i += 2) {
        const double t0 = static_cast<double>(i) / static_cast<double>(n);
        const double t1 = static_cast<double>(std::min(i + 1, n)) / static_cast<double>(n);
        drawLine3D(drawList, a + (b - a) * t0, a + (b - a) * t1, col, thickness);
    }
}

void TacticalOverlays::drawRing3D(void* drawList, const vsg::dvec3& center, const vsg::dvec3& axisU, const vsg::dvec3& axisV,
                                  double radiusU, double radiusV, int segments, std::uint32_t col, float thickness, bool dashed) const {
    const int n = std::clamp(segments, 8, 64);
    for (int i = 0; i < n; ++i) {
        if (dashed && (i & 1)) continue;
        const double a0 = (2.0 * kPi * static_cast<double>(i)) / static_cast<double>(n);
        const double a1 = (2.0 * kPi * static_cast<double>(i + 1)) / static_cast<double>(n);
        const vsg::dvec3 p0 = center + axisU * (radiusU * std::cos(a0)) + axisV * (radiusV * std::sin(a0));
        const vsg::dvec3 p1 = center + axisU * (radiusU * std::cos(a1)) + axisV * (radiusV * std::sin(a1));
        drawLine3D(drawList, p0, p1, col, thickness);
    }
}

void TacticalOverlays::drawArc3D(void* drawList, const vsg::dvec3& origin, const vsg::dvec3& fwd, const vsg::dvec3& right,
                                 double radiusM, double azStart, double azEnd, int segments, std::uint32_t col, float thickness, bool dashed) const {
    const int n = std::clamp(segments, 4, 48);
    for (int i = 0; i < n; ++i) {
        if (dashed && (i & 1)) continue;
        const double a0 = azStart + (azEnd - azStart) * (static_cast<double>(i) / static_cast<double>(n));
        const double a1 = azStart + (azEnd - azStart) * (static_cast<double>(i + 1) / static_cast<double>(n));
        const vsg::dvec3 p0 = origin + (fwd * std::cos(a0) + right * std::sin(a0)) * radiusM;
        const vsg::dvec3 p1 = origin + (fwd * std::cos(a1) + right * std::sin(a1)) * radiusM;
        drawLine3D(drawList, p0, p1, col, thickness);
    }
}

void TacticalOverlays::drawRadarVolumes(void* drawList) const {
    auto* dl = static_cast<ImDrawList*>(drawList);
    const float k = ImGui::GetIO().FontGlobalScale;
    const bool showJamBurn = (controls_->tacticalOverlayMask.load(std::memory_order_relaxed) & kOverlayJammer) != 0u;

    for (const auto& r : frame_.radars) {
        if (r.slot >= states_.size() || r.slot >= alive_.size() || !alive_[r.slot]) continue;
        const sim::VehicleState& st = states_[r.slot];
        const vsg::dvec3 origin(st.positionEcef[0], st.positionEcef[1], st.positionEcef[2]);
        const vsg::dvec3 bFwd = bodyToEcefVec(st, vsg::dvec3(1.0, 0.0, 0.0));
        const vsg::dvec3 bRight = bodyToEcefVec(st, vsg::dvec3(0.0, 1.0, 0.0));
        const vsg::dvec3 bUp = bodyToEcefVec(st, vsg::dvec3(0.0, 0.0, -1.0));

        const bool stt = (r.flags & ipc::kTactRadarSttLock) != 0u;
        const bool illum = (r.flags & ipc::kTactRadarIlluminating) != 0u;
        const bool roto = (r.flags & ipc::kTactRadarRotodome) != 0u;
        const bool jammed = (r.flags & ipc::kTactRadarJammed) != 0u;

        const double nomRange = std::clamp(static_cast<double>(r.detectionRangeM > 100.0f ? r.detectionRangeM : r.rangeScaleM), 5000.0, 350000.0);
        const ImU32 wedgeCol = illum ? IM_COL32(255, 70, 50, 190)
                               : stt ? IM_COL32(255, 175, 40, 175)
                               : roto ? IM_COL32(70, 220, 255, 130)
                                      : IM_COL32(60, 210, 255, 135);
        const ImU32 beamCol = illum ? IM_COL32(255, 60, 40, 240)
                              : stt ? IM_COL32(255, 200, 60, 235)
                                    : IM_COL32(130, 245, 255, 220);

        if (roto) {
            vsg::dvec3 e, n, u;
            localEnu(origin, e, n, u);
            drawRing3D(dl, origin, e, n, nomRange, nomRange, 48, wedgeCol, 1.4f * k, true);
            drawRing3D(dl, origin, e, n, nomRange * 0.5, nomRange * 0.5, 36, IM_COL32(70, 220, 255, 70), 1.0f * k, true);
        } else {
            const double az0 = static_cast<double>(r.azimuthCentreRad) - static_cast<double>(r.azimuthHalfWidthRad);
            const double az1 = static_cast<double>(r.azimuthCentreRad) + static_cast<double>(r.azimuthHalfWidthRad);
            const double el0 = static_cast<double>(r.elevationCentreRad) - static_cast<double>(r.elevationHalfWidthRad);
            const double el1 = static_cast<double>(r.elevationCentreRad) + static_cast<double>(r.elevationHalfWidthRad);
            const double elMid = static_cast<double>(r.elevationCentreRad);

            auto dirAt = [&](double az, double el) {
                return bFwd * (std::cos(el) * std::cos(az)) + bRight * (std::cos(el) * std::sin(az)) + bUp * std::sin(el);
            };

            // Upper & lower scan pyramid edges and range arcs (RCS-aware detection envelope)
            const vsg::dvec3 pTL = origin + dirAt(az0, el1) * nomRange;
            const vsg::dvec3 pTR = origin + dirAt(az1, el1) * nomRange;
            const vsg::dvec3 pBL = origin + dirAt(az0, el0) * nomRange;
            const vsg::dvec3 pBR = origin + dirAt(az1, el0) * nomRange;
            drawLine3D(dl, origin, pTL, wedgeCol, 1.3f * k);
            drawLine3D(dl, origin, pTR, wedgeCol, 1.3f * k);
            drawLine3D(dl, origin, pBL, wedgeCol, 1.3f * k);
            drawLine3D(dl, origin, pBR, wedgeCol, 1.3f * k);
            drawLine3D(dl, pTL, pBL, wedgeCol, 1.1f * k);
            drawLine3D(dl, pTR, pBR, wedgeCol, 1.1f * k);

            const vsg::dvec3 midFwd = bFwd * std::cos(elMid) + bUp * std::sin(elMid);
            drawArc3D(dl, origin, midFwd, bRight, nomRange, az0, az1, 20, wedgeCol, 1.6f * k, jammed);
            drawArc3D(dl, origin, bFwd * std::cos(el1) + bUp * std::sin(el1), bRight, nomRange, az0, az1, 16, IM_COL32(60, 210, 255, 85), 1.0f * k, true);
            drawArc3D(dl, origin, bFwd * std::cos(el0) + bUp * std::sin(el0), bRight, nomRange, az0, az1, 16, IM_COL32(60, 210, 255, 85), 1.0f * k, true);
            drawArc3D(dl, origin, midFwd, bRight, nomRange * 0.5, az0, az1, 14, IM_COL32(60, 210, 255, 65), 1.0f * k, true);
        }

        // Instantaneous scanning beam ray
        const double bAz = static_cast<double>(r.beamAzimuthRad);
        const double bEl = static_cast<double>(r.beamElevationRad);
        const vsg::dvec3 beamDir = bFwd * (std::cos(bEl) * std::cos(bAz)) + bRight * (std::cos(bEl) * std::sin(bAz)) + bUp * std::sin(bEl);
        const double beamLen = jammed ? std::max(3000.0, static_cast<double>(r.burnThroughRangeM)) : nomRange;
        drawLine3D(dl, origin, origin + beamDir * beamLen, beamCol, 2.0f * k);

        // Live Burn-Through Pocket inside unjammed envelope when jammed (shown whenever Radar or Jammer layer is active)
        if (jammed && r.burnThroughRangeM > 100.0f && r.burnThroughRangeM < r.detectionRangeM * 0.96f) {
            const ImU32 btCol = showJamBurn ? IM_COL32(255, 110, 230, 230) : IM_COL32(255, 165, 50, 215);
            const double btM = static_cast<double>(r.burnThroughRangeM);
            if (roto) {
                vsg::dvec3 e, n, u;
                localEnu(origin, e, n, u);
                drawRing3D(dl, origin, e, n, btM, btM, 40, btCol, 2.2f * k, false);
            } else {
                const double az0 = static_cast<double>(r.azimuthCentreRad) - static_cast<double>(r.azimuthHalfWidthRad);
                const double az1 = static_cast<double>(r.azimuthCentreRad) + static_cast<double>(r.azimuthHalfWidthRad);
                const vsg::dvec3 midFwd = bFwd * std::cos(r.elevationCentreRad) + bUp * std::sin(r.elevationCentreRad);
                drawArc3D(dl, origin, midFwd, bRight, btM, az0, az1, 20, btCol, 2.4f * k, false);
            }
            const vsg::dvec3 tagPos = origin + beamDir * btM;
            float tx = 0.0f, ty = 0.0f;
            if (projectPoint(tagPos, tx, ty)) {
                char buf[64];
                std::snprintf(buf, sizeof buf, "BURN-THRU %.1f km (-%.1f dB J/N)", btM * 1e-3, static_cast<double>(r.jammerRatioDb));
                drawTag(dl, tx + 6.0f * k, ty - 8.0f * k, buf, IM_COL32(255, 220, 250, 255), IM_COL32(85, 16, 75, 205), k);
            }
        }

        // STT / Illumination / Track lock line & target marker
        if ((r.flags & ipc::kTactRadarHasTarget) != 0u) {
            const vsg::dvec3 tgt(r.targetEcef[0], r.targetEcef[1], r.targetEcef[2]);
            if (stt || illum) {
                drawLine3D(dl, origin, tgt, beamCol, 2.2f * k);
            } else {
                drawDashedLine3D(dl, origin, tgt, IM_COL32(100, 235, 255, 160), 1.3f * k, 18);
            }
            float sx = 0.0f, sy = 0.0f;
            if (projectPoint(tgt, sx, sy)) {
                const float rad = (stt ? 11.0f : 8.0f) * k;
                const ImVec2 pts[4] = {{sx, sy - rad}, {sx + rad, sy}, {sx, sy + rad}, {sx - rad, sy}};
                dl->AddPolyline(pts, 4, beamCol, ImDrawFlags_Closed, 1.8f * k);
                char buf[48];
                std::snprintf(buf, sizeof buf, "%s %.1f km", illum ? "ILLUM" : stt ? "STT" : "TWS",
                              static_cast<double>(r.targetRangeM) * 1e-3);
                drawTag(dl, sx + rad + 4.0f * k, sy - 7.0f * k, buf, IM_COL32(255, 255, 255, 255),
                        illum ? IM_COL32(180, 25, 25, 210) : stt ? IM_COL32(170, 95, 15, 205) : IM_COL32(15, 75, 105, 195), k);
            }
        }
    }
}

void TacticalOverlays::drawJammerLobes(void* drawList) const {
    auto* dl = static_cast<ImDrawList*>(drawList);
    const float k = ImGui::GetIO().FontGlobalScale;

    for (const auto& j : frame_.jammers) {
        const vsg::dvec3 origin(j.originEcef[0], j.originEcef[1], j.originEcef[2]);
        vsg::dvec3 dir(j.boresightEcef[0], j.boresightEcef[1], j.boresightEcef[2]);
        const double dn = vsg::length(dir);
        if (dn < 1e-6) continue;
        dir /= dn;

        const bool allRound = (j.flags & ipc::kTactJamAllRound) != 0u;
        const bool comms = (j.flags & ipc::kTactJamComms) != 0u;
        const ImU32 col = comms ? IM_COL32(255, 145, 50, 175) : IM_COL32(225, 65, 245, 175);
        const ImU32 colDim = comms ? IM_COL32(255, 145, 50, 95) : IM_COL32(225, 65, 245, 95);
        const double rng = static_cast<double>(j.rangeM);

        if (allRound) {
            vsg::dvec3 e, n, u;
            localEnu(origin, e, n, u);
            drawRing3D(dl, origin, e, n, rng * 0.45, rng * 0.45, 36, col, 1.6f * k, false);
            drawRing3D(dl, origin, e, n, rng, rng, 44, colDim, 1.3f * k, true);
            drawRing3D(dl, origin, e, u, rng * 0.45, rng * 0.45, 28, colDim, 1.1f * k, true);
        } else {
            vsg::dvec3 u, v;
            orthoBasis(dir, u, v);
            const double halfAng = std::clamp(static_cast<double>(j.halfAngleRad), 0.05, 1.35);
            const double tanH = std::tan(halfAng);
            // 3 expanding jammer lobe cross-section rings + 8 cone ribs
            for (const double frac : {0.35, 0.70, 1.0}) {
                const double d = rng * frac;
                const double rad = d * tanH;
                drawRing3D(dl, origin + dir * d, u, v, rad, rad, 28, frac > 0.9 ? col : colDim, (frac > 0.9 ? 1.8f : 1.1f) * k, frac < 0.9);
            }
            const double rFar = rng * tanH;
            for (int r = 0; r < 8; ++r) {
                const double ang = (2.0 * kPi * r) / 8.0;
                const vsg::dvec3 rim = origin + dir * rng + (u * std::cos(ang) + v * std::sin(ang)) * rFar;
                drawLine3D(dl, origin, rim, col, 1.2f * k);
            }
        }

        const vsg::dvec3 lblPos = allRound ? origin : (origin + dir * (rng * 0.55));
        float sx = 0.0f, sy = 0.0f;
        if (projectPoint(lblPos, sx, sy)) {
            char buf[64];
            std::snprintf(buf, sizeof buf, "%s %.0f dBW | %.2f GHz", comms ? "EA COMMS" : "EA JAM",
                          static_cast<double>(j.erpDbw), static_cast<double>(j.frequencyGhz));
            drawTag(dl, sx + 6.0f * k, sy - 10.0f * k, buf, IM_COL32(255, 235, 255, 255),
                    comms ? IM_COL32(135, 65, 10, 200) : IM_COL32(115, 20, 130, 200), k);
        }
    }
}

void TacticalOverlays::drawEsmRays(void* drawList) const {
    auto* dl = static_cast<ImDrawList*>(drawList);
    const float k = ImGui::GetIO().FontGlobalScale;

    for (const auto& e : frame_.esmRays) {
        const vsg::dvec3 origin(e.originEcef[0], e.originEcef[1], e.originEcef[2]);
        const vsg::dvec3 dir(e.rayDirEcef[0], e.rayDirEcef[1], e.rayDirEcef[2]);
        const bool warn = (e.flags & ipc::kTactEsmWarning) != 0u;
        const bool hasGeo = (e.flags & ipc::kTactEsmHasGeo) != 0u;
        const bool comint = (e.flags & ipc::kTactEsmComint) != 0u;

        const ImU32 rayCol = warn ? IM_COL32(255, 60, 60, 230)
                             : comint ? IM_COL32(80, 250, 175, 185)
                                      : IM_COL32(255, 225, 70, 175);
        const vsg::dvec3 tip = hasGeo ? vsg::dvec3(e.geoPosEcef[0], e.geoPosEcef[1], e.geoPosEcef[2])
                                      : (origin + dir * static_cast<double>(e.rayLengthM));
        if (hasGeo) drawLine3D(dl, origin, tip, rayCol, 1.6f * k);
        else drawDashedLine3D(dl, origin, tip, rayCol, 1.3f * k, 18);

        if (hasGeo && e.crlbCep50M > 1.0f) {
            const vsg::dvec3 geo(e.geoPosEcef[0], e.geoPosEcef[1], e.geoPosEcef[2]);
            vsg::dvec3 east, north, up;
            localEnu(geo, east, north, up);
            const double rx = std::clamp(static_cast<double>(e.crlbCep50M), 50.0, 50000.0);
            const double rz = std::clamp(static_cast<double>(e.crlbVertM > 1.0f ? e.crlbVertM : e.crlbCep50M * 0.65f), 40.0, 35000.0);
            const ImU32 ellCol = warn ? IM_COL32(255, 90, 90, 225) : IM_COL32(255, 215, 60, 215);
            // 3 orthogonal ellipses forming the 3-D Gauss-Newton FIM CRLB uncertainty ellipsoid
            drawRing3D(dl, geo, east, north, rx, rx, 28, ellCol, 2.0f * k, false);
            drawRing3D(dl, geo, east, up, rx, rz, 24, IM_COL32(255, 215, 60, 145), 1.3f * k, true);
            drawRing3D(dl, geo, north, up, rx, rz, 24, IM_COL32(255, 215, 60, 145), 1.3f * k, true);
            float sx = 0.0f, sy = 0.0f;
            if (projectPoint(geo, sx, sy)) {
                char buf[48];
                if (rx >= 1000.0) std::snprintf(buf, sizeof buf, "CRLB CEP %.2f km", rx * 1e-3);
                else std::snprintf(buf, sizeof buf, "CRLB CEP %.0f m", rx);
                drawTag(dl, sx + 8.0f * k, sy - 8.0f * k, buf, IM_COL32(255, 250, 200, 255), IM_COL32(85, 65, 10, 210), k);
            }
        }
    }
}

void TacticalOverlays::drawLinkPulses(void* drawList) const {
    auto* dl = static_cast<ImDrawList*>(drawList);
    const float k = ImGui::GetIO().FontGlobalScale;

    for (const auto& l : frame_.links) {
        const auto kind = static_cast<ipc::TacticalLinkKindV1>(l.kind);
        const vsg::dvec3 from(l.fromEcef[0], l.fromEcef[1], l.fromEcef[2]);
        const vsg::dvec3 to(l.toEcef[0], l.toEcef[1], l.toEcef[2]);
        const double phase = std::clamp(static_cast<double>(l.ageS) / std::max(0.1, static_cast<double>(l.durationS)), 0.0, 1.0);

        if (kind == ipc::TacticalLinkKindV1::Link16Dms || kind == ipc::TacticalLinkKindV1::C2Consent ||
            kind == ipc::TacticalLinkKindV1::Mode5IffReply) {
            const ImU32 arcCol = (kind == ipc::TacticalLinkKindV1::C2Consent) ? IM_COL32(255, 205, 50, 210)
                                 : (kind == ipc::TacticalLinkKindV1::Mode5IffReply) ? IM_COL32(140, 240, 255, 175)
                                                                                    : IM_COL32(65, 245, 165, 180);
            drawDashedLine3D(dl, from, to, arcCol, 1.5f * k, 16);
            // Animated traveling packet bead along link vector
            const vsg::dvec3 pkt = from + (to - from) * phase;
            float sx = 0.0f, sy = 0.0f;
            if (projectPoint(pkt, sx, sy)) {
                dl->AddCircleFilled(ImVec2(sx, sy), 4.5f * k, arcCol);
                const char* tag = (kind == ipc::TacticalLinkKindV1::C2Consent) ? "C2 CONSENT"
                                  : (kind == ipc::TacticalLinkKindV1::Mode5IffReply) ? "M5 1090" : "L16";
                drawTag(dl, sx + 7.0f * k, sy - 7.0f * k, tag, IM_COL32(240, 255, 250, 255), IM_COL32(15, 65, 55, 195), k);
            }
        } else if (kind == ipc::TacticalLinkKindV1::Mode5IffInterrogate) {
            vsg::dvec3 e, n, u;
            localEnu(from, e, n, u);
            const double rad = std::max(500.0, static_cast<double>(l.param) * phase);
            const std::uint8_t alpha = static_cast<std::uint8_t>(std::clamp(220.0 * (1.0 - phase), 35.0, 220.0));
            const ImU32 col = (l.flags & ipc::kTactLinkJammed) ? IM_COL32(255, 120, 60, alpha) : IM_COL32(90, 220, 255, alpha);
            drawRing3D(dl, from, e, n, rad, rad, 36, col, 1.8f * k, false);
        } else if (kind == ipc::TacticalLinkKindV1::PntGnssDrift) {
            vsg::dvec3 e, n, u;
            localEnu(from, e, n, u);
            const bool jammed = (l.flags & ipc::kTactLinkJammed) != 0u;
            const bool locked = (l.flags & ipc::kTactLinkGnssLocked) != 0u;
            const double errM = static_cast<double>(l.param);
            const double drawR = std::max(errM, 40.0);
            const ImU32 col = jammed ? IM_COL32(255, 70, 50, 230)
                              : locked ? IM_COL32(60, 240, 120, 175)
                                       : IM_COL32(120, 195, 255, 185);
            drawRing3D(dl, from, e, n, drawR, drawR, 28, col, jammed ? 2.2f * k : 1.4f * k, !jammed);
            float sx = 0.0f, sy = 0.0f;
            if (projectPoint(from + e * drawR, sx, sy)) {
                char buf[48];
                std::snprintf(buf, sizeof buf, "%s %.0f m", jammed ? "GNSS JAM" : locked ? "GPS" : "INS DRIFT", errM);
                drawTag(dl, sx + 5.0f * k, sy - 6.0f * k, buf, IM_COL32(255, 255, 255, 255),
                        jammed ? IM_COL32(170, 25, 20, 215) : IM_COL32(20, 60, 80, 195), k);
            }
        }
    }
}

void TacticalOverlays::drawWeaponCues(void* drawList) const {
    auto* dl = static_cast<ImDrawList*>(drawList);
    const float k = ImGui::GetIO().FontGlobalScale;

    for (const auto& w : frame_.weapons) {
        if (w.slot >= states_.size() || w.slot >= alive_.size() || !alive_[w.slot]) continue;
        const sim::VehicleState& st = states_[w.slot];
        const vsg::dvec3 origin(st.positionEcef[0], st.positionEcef[1], st.positionEcef[2]);
        const vsg::dvec3 bFwd = bodyToEcefVec(st, vsg::dvec3(1.0, 0.0, 0.0));
        const vsg::dvec3 bRight = bodyToEcefVec(st, vsg::dvec3(0.0, 1.0, 0.0));

        // 1. Dynamic 5-D launchZoneV2 DLZ/WEZ arcs toward tracked target
        if ((w.flags & ipc::kTactWeaponHasDlz) != 0u && w.rMaxM > 100.0f) {
            const vsg::dvec3 tgt(w.targetEcef[0], w.targetEcef[1], w.targetEcef[2]);
            vsg::dvec3 toTgt = tgt - origin;
            const double dist = vsg::length(toTgt);
            if (dist > 10.0) {
                vsg::dvec3 dir = toTgt / dist;
                vsg::dvec3 e, n, up;
                localEnu(origin, e, n, up);
                vsg::dvec3 right = vsg::cross(dir, up);
                if (vsg::length(right) < 1e-6) right = bRight;
                right = vsg::normalize(right);

                const double span = 0.28; // +/-16 deg WEZ fan centered on target bearing
                const bool inNez = (w.flags & ipc::kTactWeaponInNez) != 0u;
                const bool inRng = (w.flags & ipc::kTactWeaponInRange) != 0u;
                if (w.rAeroM > w.rMaxM)
                    drawArc3D(dl, origin, dir, right, w.rAeroM, -span, span, 14, IM_COL32(160, 210, 180, 105), 1.1f * k, true);
                drawArc3D(dl, origin, dir, right, w.rMaxM, -span, span, 16,
                          inRng ? IM_COL32(90, 255, 110, 210) : IM_COL32(255, 200, 60, 180), 1.8f * k, false);
                if (w.rNeM > 100.0f) {
                    const ImU32 nezCol = inNez ? IM_COL32(255, 65, 55, 240) : IM_COL32(255, 130, 50, 205);
                    drawArc3D(dl, origin, dir, right, w.rNeM, -span, span, 16, nezCol, 2.5f * k, false);
                    drawLine3D(dl, origin + (dir * std::cos(-span) + right * std::sin(-span)) * w.rMinM,
                               origin + (dir * std::cos(-span) + right * std::sin(-span)) * w.rMaxM, IM_COL32(255, 195, 70, 135), 1.2f * k);
                    drawLine3D(dl, origin + (dir * std::cos(span) + right * std::sin(span)) * w.rMinM,
                               origin + (dir * std::cos(span) + right * std::sin(span)) * w.rMaxM, IM_COL32(255, 195, 70, 135), 1.2f * k);
                }
                if (w.rMinM > 50.0f)
                    drawArc3D(dl, origin, dir, right, w.rMinM, -span, span, 10, IM_COL32(255, 90, 90, 175), 1.4f * k, true);

                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(origin + dir * w.rMaxM, sx, sy)) {
                    char buf[64];
                    std::snprintf(buf, sizeof buf, "%s Rmax %.0f | Rne %.0f km%s",
                                  w.storeName[0] ? w.storeName : "AAM",
                                  w.rMaxM * 1e-3, w.rNeM * 1e-3,
                                  inNez ? " [NEZ]" : inRng ? " [IN RNG]" : "");
                    drawTag(dl, sx + 6.0f * k, sy - 8.0f * k, buf, IM_COL32(240, 255, 240, 255),
                            inNez ? IM_COL32(165, 25, 25, 215) : IM_COL32(20, 75, 35, 200), k);
                }
            }
        }

        // 2. McCoy / STANAG 4355 Gun LCOS Funnel & Lead Pipper
        if ((w.flags & ipc::kTactWeaponHasLcos) != 0u) {
            const vsg::dvec3 pip(w.lcosPipperEcef[0], w.lcosPipperEcef[1], w.lcosPipperEcef[2]);
            const double spanHalfM = 5.5; // Fighter reference wingspan funnel rails
            const vsg::dvec3 startL = origin + bFwd * 35.0 - bRight * 1.0;
            const vsg::dvec3 startR = origin + bFwd * 35.0 + bRight * 1.0;
            drawLine3D(dl, startL, pip - bRight * spanHalfM, IM_COL32(80, 255, 120, 185), 1.4f * k);
            drawLine3D(dl, startR, pip + bRight * spanHalfM, IM_COL32(80, 255, 120, 185), 1.4f * k);
            float px = 0.0f, py = 0.0f;
            if (projectPoint(pip, px, py)) {
                dl->AddCircle(ImVec2(px, py), 11.0f * k, IM_COL32(80, 255, 120, 240), 20, 1.8f * k);
                dl->AddCircleFilled(ImVec2(px, py), 2.5f * k, IM_COL32(80, 255, 120, 255));
                drawTag(dl, px + 14.0f * k, py - 7.0f * k, "LCOS", IM_COL32(120, 255, 150, 255), IM_COL32(12, 45, 20, 195), k);
            }
        }

        // 3. Wind-Compensated Mk 82 CCIP Bomb Fall Line & Ground Crosshair
        if ((w.flags & ipc::kTactWeaponHasCcip) != 0u) {
            const vsg::dvec3 impact(w.ccipImpactEcef[0], w.ccipImpactEcef[1], w.ccipImpactEcef[2]);
            drawDashedLine3D(dl, origin, impact, IM_COL32(100, 255, 140, 195), 1.5f * k, 18);
            vsg::dvec3 e, n, u;
            localEnu(impact, e, n, u);
            drawRing3D(dl, impact, e, n, 22.0, 22.0, 24, IM_COL32(95, 255, 125, 235), 2.0f * k, false);
            drawLine3D(dl, impact - e * 38.0, impact + e * 38.0, IM_COL32(95, 255, 125, 235), 1.8f * k);
            drawLine3D(dl, impact - n * 38.0, impact + n * 38.0, IM_COL32(95, 255, 125, 235), 1.8f * k);
            float ix = 0.0f, iy = 0.0f;
            if (projectPoint(impact, ix, iy)) {
                char buf[48];
                std::snprintf(buf, sizeof buf, "CCIP %s", w.storeName[0] ? w.storeName : "Mk 82");
                drawTag(dl, ix + 10.0f * k, iy - 8.0f * k, buf, IM_COL32(150, 255, 170, 255), IM_COL32(15, 55, 25, 210), k);
            }
        }
    }
}

void TacticalOverlays::drawSwaths(void* drawList) const {
    auto* dl = static_cast<ImDrawList*>(drawList);
    const float k = ImGui::GetIO().FontGlobalScale;

    for (const auto& sw : frame_.swaths) {
        const auto kind = static_cast<ipc::TacticalSwathKindV1>(sw.kind);
        const vsg::dvec3 center(sw.centerEcef[0], sw.centerEcef[1], sw.centerEcef[2]);
        const vsg::dvec3 east(sw.axisEastEcef[0], sw.axisEastEcef[1], sw.axisEastEcef[2]);
        const vsg::dvec3 north(sw.axisNorthEcef[0], sw.axisNorthEcef[1], sw.axisNorthEcef[2]);
        const double hw = static_cast<double>(sw.halfWidthM);
        const double hl = static_cast<double>(sw.halfLengthM);

        const vsg::dvec3 corners[4] = {
            center - east * hw - north * hl,
            center + east * hw - north * hl,
            center + east * hw + north * hl,
            center - east * hw + north * hl};

        const bool jammed = (sw.flags & ipc::kTactSwathJammed) != 0u;
        const ImU32 lineCol = (kind == ipc::TacticalSwathKindV1::SarSpotlight)
                                  ? (jammed ? IM_COL32(255, 160, 45, 230) : IM_COL32(75, 255, 185, 225))
                                  : weatherDbzColor(sw.metricDb, 220);
        const ImU32 fillCol = (kind == ipc::TacticalSwathKindV1::SarSpotlight)
                                  ? (jammed ? IM_COL32(255, 150, 35, 42) : IM_COL32(60, 240, 175, 42))
                                  : weatherDbzColor(sw.metricDb, 65);

        ImVec2 projCorners[4];
        bool allProjected = true;
        for (int i = 0; i < 4; ++i) {
            drawLine3D(dl, corners[i], corners[(i + 1) & 3], lineCol, 2.0f * k);
            if (!projectPoint(corners[i], projCorners[i].x, projCorners[i].y)) allProjected = false;
        }
        if (allProjected) dl->AddConvexPolyFilled(projCorners, 4, fillCol);

        // Connect SAR platform to its ground spotlight footprint
        if (kind == ipc::TacticalSwathKindV1::SarSpotlight && sw.slot < states_.size() && sw.slot < alive_.size() && alive_[sw.slot]) {
            const vsg::dvec3 orig(states_[sw.slot].positionEcef[0], states_[sw.slot].positionEcef[1], states_[sw.slot].positionEcef[2]);
            drawDashedLine3D(dl, orig, center, IM_COL32(75, 255, 185, 140), 1.3f * k, 16);
        }

        float cx = 0.0f, cy = 0.0f;
        if (projectPoint(center, cx, cy)) {
            char buf[64];
            if (kind == ipc::TacticalSwathKindV1::SarSpotlight) {
                std::snprintf(buf, sizeof buf, "SAR %.1fm | NESZ %.1f dB%s",
                              static_cast<double>(sw.resolutionM), static_cast<double>(sw.metricDb),
                              jammed ? " [JAM]" : "");
                drawTag(dl, cx + 6.0f * k, cy - 8.0f * k, buf, IM_COL32(215, 255, 235, 255), IM_COL32(12, 65, 50, 205), k);
            } else {
                std::snprintf(buf, sizeof buf, "WX %.0f dBZ", static_cast<double>(sw.metricDb));
                drawTag(dl, cx + 6.0f * k, cy - 8.0f * k, buf, IM_COL32(255, 255, 255, 255), IM_COL32(30, 35, 45, 200), k);
            }
        }
    }
}

void TacticalOverlays::drawCockpitHud(void* drawList) const {
    if (selectedSlot_ < 0 || static_cast<std::size_t>(selectedSlot_) >= states_.size() ||
        static_cast<std::size_t>(selectedSlot_) >= alive_.size() || !alive_[static_cast<std::size_t>(selectedSlot_)]) {
        return;
    }
    auto* dl = static_cast<ImDrawList*>(drawList);
    const auto& st = states_[static_cast<std::size_t>(selectedSlot_)];
    const float k = ImGui::GetIO().FontGlobalScale;
    const float W = static_cast<float>(width_);
    const float H = static_cast<float>(height_);
    const float cx = W * 0.5f;
    const float cy = H * 0.5f;
    const ImU32 hud = IM_COL32(65, 255, 115, 230);
    const ImU32 hudDim = IM_COL32(65, 255, 115, 140);
    const ImU32 hudWarn = IM_COL32(255, 75, 55, 240);

    // 1. Waterline boresight (-v-) at projected aircraft forward vector
    const vsg::dvec3 ownPos(st.positionEcef[0], st.positionEcef[1], st.positionEcef[2]);
    const vsg::dvec3 fwd = bodyToEcefVec(st, vsg::dvec3(1.0, 0.0, 0.0));
    float bx = cx, by = cy;
    if (projectPoint(ownPos + fwd * 5000.0, bx, by)) {
        const float w = 14.0f * k;
        dl->AddLine(ImVec2(bx - 1.5f * w, by), ImVec2(bx - 0.5f * w, by), hud, 1.8f * k);
        dl->AddLine(ImVec2(bx - 0.5f * w, by), ImVec2(bx, by + 0.45f * w), hud, 1.8f * k);
        dl->AddLine(ImVec2(bx, by + 0.45f * w), ImVec2(bx + 0.5f * w, by), hud, 1.8f * k);
        dl->AddLine(ImVec2(bx + 0.5f * w, by), ImVec2(bx + 1.5f * w, by), hud, 1.8f * k);
    }

    // 2. Flight Path Vector (FPV) from actual ECEF velocity
    const vsg::dvec3 velEcef = bodyToEcefVec(st, vsg::dvec3(st.velocityBodyMs[0], st.velocityBodyMs[1], st.velocityBodyMs[2]));
    const double speed = vsg::length(velEcef);
    if (speed > 5.0) {
        float fx = cx, fy = cy;
        if (projectPoint(ownPos + (velEcef / speed) * 5000.0, fx, fy)) {
            const float r = 9.0f * k;
            dl->AddCircle(ImVec2(fx, fy), r, hud, 20, 1.8f * k);
            dl->AddLine(ImVec2(fx - r, fy), ImVec2(fx - 2.2f * r, fy), hud, 1.8f * k);
            dl->AddLine(ImVec2(fx + r, fy), ImVec2(fx + 2.2f * r, fy), hud, 1.8f * k);
            dl->AddLine(ImVec2(fx, fy - r), ImVec2(fx, fy - 1.9f * r), hud, 1.8f * k);
        }
    }

    // 3. Banked Pitch Ladder (-30..+30 deg in 5 deg increments)
    const double rollRad = st.eulerRad[0];
    const double pitchDeg = units::radiansToDegrees(st.eulerRad[1]);
    const float cosR = static_cast<float>(std::cos(-rollRad));
    const float sinR = static_cast<float>(std::sin(-rollRad));
    const float pxPerDeg = H / 30.0f; // matches 30 deg vertical FOV
    auto rotPt = [&](float lx, float ly) -> ImVec2 {
        return ImVec2(cx + lx * cosR - ly * sinR, cy + lx * sinR + ly * cosR);
    };
    for (int deg = -30; deg <= 30; deg += 5) {
        const float dy = static_cast<float>((pitchDeg - static_cast<double>(deg))) * pxPerDeg;
        if (std::abs(dy) > H * 0.28f) continue;
        const float inner = (deg == 0 ? 48.0f : 38.0f) * k;
        const float outer = (deg == 0 ? 155.0f : 96.0f) * k;
        const float tick = (deg >= 0 ? 10.0f : -10.0f) * k;
        for (float side : {-1.0f, 1.0f}) {
            const ImVec2 p0 = rotPt(side * inner, dy);
            const ImVec2 p1 = rotPt(side * outer, dy);
            if (deg >= 0) {
                dl->AddLine(p0, p1, deg == 0 ? hud : hudDim, (deg == 0 ? 2.0f : 1.5f) * k);
            } else {
                const ImVec2 pm = rotPt(side * (0.5f * (inner + outer)), dy);
                dl->AddLine(p0, rotPt(side * (inner + 0.35f * (outer - inner)), dy), hudDim, 1.5f * k);
                dl->AddLine(pm, p1, hudDim, 1.5f * k);
            }
            if (deg != 0) {
                dl->AddLine(p0, rotPt(side * inner, dy + tick), hudDim, 1.5f * k);
                char num[12];
                std::snprintf(num, sizeof num, "%d", std::abs(deg));
                const ImVec2 tp = rotPt(side * (outer + 14.0f * k), dy - 6.0f * k);
                dl->AddText(tp, hudDim, num);
            }
        }
    }

    // 4. Left Airspeed / Mach / G / AoA Tape & Right Altitude / VVI Tape
    const float tapeOffset = std::min(W * 0.24f, 340.0f * k);
    const float tapeH = 220.0f * k;
    const float lx = cx - tapeOffset;
    const float rx = cx + tapeOffset;
    dl->AddLine(ImVec2(lx, cy - tapeH * 0.5f), ImVec2(lx, cy + tapeH * 0.5f), hudDim, 1.5f * k);
    dl->AddLine(ImVec2(rx, cy - tapeH * 0.5f), ImVec2(rx, cy + tapeH * 0.5f), hudDim, 1.5f * k);

    char buf[64];
    const double iasKt = units::metresPerSecondToKnots(st.airspeedCalibratedMs > 0.1 ? st.airspeedCalibratedMs : st.airspeedTrueMs);
    std::snprintf(buf, sizeof buf, "%3.0f KT", iasKt);
    dl->AddRect(ImVec2(lx - 78.0f * k, cy - 13.0f * k), ImVec2(lx - 6.0f * k, cy + 13.0f * k), hud, 0.0f, 0, 1.6f * k);
    dl->AddText(ImVec2(lx - 70.0f * k, cy - 8.0f * k), hud, buf);

    const double soundMs = std::max(295.0, 340.3 - 0.004 * std::clamp(st.altitudeMslM, 0.0, 11000.0));
    const double mach = st.airspeedTrueMs / soundMs;
    const double gLoad = std::isfinite(st.loadFactor) ? st.loadFactor : 1.0;
    std::snprintf(buf, sizeof buf, "M %.2f  G %.1f  A %.1f", mach, gLoad, units::radiansToDegrees(st.alphaRad));
    dl->AddText(ImVec2(lx - 86.0f * k, cy + tapeH * 0.5f + 12.0f * k), hud, buf);

    std::snprintf(buf, sizeof buf, "%5.0f m", st.altitudeMslM);
    dl->AddRect(ImVec2(rx + 6.0f * k, cy - 13.0f * k), ImVec2(rx + 86.0f * k, cy + 13.0f * k), hud, 0.0f, 0, 1.6f * k);
    dl->AddText(ImVec2(rx + 12.0f * k, cy - 8.0f * k), hud, buf);
    std::snprintf(buf, sizeof buf, "RALT %.0f m  VVI %+.0f", st.altitudeAglM, -st.velocityNedMs[2]);
    dl->AddText(ImVec2(rx - 10.0f * k, cy + tapeH * 0.5f + 12.0f * k), hud, buf);

    // 5. Top Heading Tape
    const float hdgY = std::max(54.0f * k, cy - H * 0.34f);
    double hdgDeg = units::radiansToDegrees(st.eulerRad[2]);
    while (hdgDeg < 0.0) hdgDeg += 360.0;
    while (hdgDeg >= 360.0) hdgDeg -= 360.0;
    dl->AddLine(ImVec2(cx - 150.0f * k, hdgY), ImVec2(cx + 150.0f * k, hdgY), hudDim, 1.5f * k);
    std::snprintf(buf, sizeof buf, "%03.0f", hdgDeg);
    dl->AddRect(ImVec2(cx - 24.0f * k, hdgY - 26.0f * k), ImVec2(cx + 24.0f * k, hdgY - 4.0f * k), hud, 0.0f, 0, 1.6f * k);
    dl->AddText(ImVec2(cx - 14.0f * k, hdgY - 22.0f * k), hud, buf);
    for (int delta = -25; delta <= 25; delta += 5) {
        const int tickDeg = (static_cast<int>(std::round(hdgDeg / 5.0)) * 5 + delta + 360) % 360;
        const float tx = cx + static_cast<float>(tickDeg - hdgDeg + (tickDeg - hdgDeg > 180.0 ? -360.0 : tickDeg - hdgDeg < -180.0 ? 360.0 : 0.0)) * 5.2f * k;
        if (std::abs(tx - cx) > 145.0f * k) continue;
        const bool major = (tickDeg % 10) == 0;
        dl->AddLine(ImVec2(tx, hdgY), ImVec2(tx, hdgY + (major ? 10.0f : 5.0f) * k), hudDim, 1.4f * k);
        if (major) {
            std::snprintf(buf, sizeof buf, "%02d", tickDeg / 10);
            dl->AddText(ImVec2(tx - 8.0f * k, hdgY + 12.0f * k), hudDim, buf);
        }
    }

    // 6. Native Tactical HUD Symbology for Own Vehicle (TD Box, DLZ Scale, LCOS, CCIP, RWR Warnings)
    if (!hasFrame_) return;
    const std::uint32_t ownSlot = static_cast<std::uint32_t>(selectedSlot_);

    // Target Designator (TD) Box from own radar track
    for (const auto& r : frame_.radars) {
        if (r.slot != ownSlot || (r.flags & ipc::kTactRadarHasTarget) == 0u) continue;
        float tx = 0.0f, ty = 0.0f;
        if (projectPoint(vsg::dvec3(r.targetEcef[0], r.targetEcef[1], r.targetEcef[2]), tx, ty)) {
            const float box = 16.0f * k;
            const ImU32 tdCol = (r.flags & ipc::kTactRadarSttLock) ? hud : hudDim;
            dl->AddRect(ImVec2(tx - box, ty - box), ImVec2(tx + box, ty + box), tdCol, 0.0f, 0, 2.0f * k);
            std::snprintf(buf, sizeof buf, "%.1f km", static_cast<double>(r.targetRangeM) * 1e-3);
            dl->AddText(ImVec2(tx - box, ty + box + 4.0f * k), tdCol, buf);
        }
        if (r.flags & ipc::kTactRadarJammed) {
            dl->AddText(ImVec2(cx - 52.0f * k, hdgY + 34.0f * k), hudWarn, "RADAR JAMMED");
        }
        break;
    }

    // Vertical HUD DLZ Bar + LCOS/CCIP Pippers
    for (const auto& w : frame_.weapons) {
        if (w.slot != ownSlot) continue;
        if ((w.flags & ipc::kTactWeaponHasDlz) != 0u && w.rAeroM > 100.0f) {
            const float dx = rx + 104.0f * k;
            const float top = cy - 95.0f * k;
            const float bot = cy + 95.0f * k;
            const float scaleM = std::max(w.rAeroM, 1000.0f);
            auto yOf = [&](float rngM) {
                return bot - std::clamp(rngM / scaleM, 0.0f, 1.0f) * (bot - top);
            };
            dl->AddLine(ImVec2(dx, top), ImVec2(dx, bot), hudDim, 1.5f * k);
            dl->AddLine(ImVec2(dx - 8.0f * k, yOf(w.rMaxM)), ImVec2(dx + 8.0f * k, yOf(w.rMaxM)), hud, 2.0f * k);
            dl->AddRectFilled(ImVec2(dx - 5.0f * k, yOf(w.rNeM)), ImVec2(dx + 5.0f * k, yOf(w.rMinM)), IM_COL32(65, 255, 115, 110));
            dl->AddRect(ImVec2(dx - 5.0f * k, yOf(w.rNeM)), ImVec2(dx + 5.0f * k, yOf(w.rMinM)), hud, 0.0f, 0, 1.5f * k);
            const double tgtRange = vsg::length(vsg::dvec3(w.targetEcef[0], w.targetEcef[1], w.targetEcef[2]) - ownPos);
            const float ty = yOf(static_cast<float>(tgtRange));
            dl->AddTriangleFilled(ImVec2(dx - 16.0f * k, ty - 5.0f * k), ImVec2(dx - 16.0f * k, ty + 5.0f * k), ImVec2(dx - 7.0f * k, ty), hud);
            dl->AddText(ImVec2(dx - 12.0f * k, bot + 6.0f * k), hud, w.storeName[0] ? w.storeName : "AAM");
            if (w.flags & ipc::kTactWeaponInNez)
                dl->AddText(ImVec2(cx - 28.0f * k, cy + 72.0f * k), hud, "SHOOT");
        }
        if ((w.flags & ipc::kTactWeaponHasLcos) != 0u) {
            float px = 0.0f, py = 0.0f;
            if (projectPoint(vsg::dvec3(w.lcosPipperEcef[0], w.lcosPipperEcef[1], w.lcosPipperEcef[2]), px, py)) {
                dl->AddCircle(ImVec2(px, py), 14.0f * k, hud, 24, 1.8f * k);
                dl->AddCircleFilled(ImVec2(px, py), 2.5f * k, hud);
            }
        }
        if ((w.flags & ipc::kTactWeaponHasCcip) != 0u) {
            float ix = 0.0f, iy = 0.0f;
            if (projectPoint(vsg::dvec3(w.ccipImpactEcef[0], w.ccipImpactEcef[1], w.ccipImpactEcef[2]), ix, iy)) {
                dl->AddLine(ImVec2(bx, by), ImVec2(ix, iy), hudDim, 1.4f * k);
                dl->AddCircle(ImVec2(ix, iy), 11.0f * k, hud, 20, 1.8f * k);
                dl->AddLine(ImVec2(ix - 16.0f * k, iy), ImVec2(ix + 16.0f * k, iy), hud, 1.5f * k);
                dl->AddLine(ImVec2(ix, iy - 16.0f * k), ImVec2(ix, iy + 16.0f * k), hud, 1.5f * k);
            }
        }
        break;
    }

    // Compact RWR Threat Azimuth Compass (bottom-right of HUD)
    int rwrCount = 0;
    for (const auto& e : frame_.esmRays)
        if (e.observerSlot == ownSlot) ++rwrCount;
    if (rwrCount > 0) {
        const float rwrR = 54.0f * k;
        const ImVec2 rc(cx + tapeOffset + 80.0f * k, cy + tapeH * 0.5f + 118.0f * k);
        dl->AddCircleFilled(rc, rwrR, IM_COL32(8, 20, 12, 165), 32);
        dl->AddCircle(rc, rwrR, hudDim, 32, 1.5f * k);
        dl->AddCircle(rc, rwrR * 0.45f, IM_COL32(65, 255, 115, 75), 24, 1.0f * k);
        dl->AddText(ImVec2(rc.x - 14.0f * k, rc.y - rwrR - 18.0f * k), hud, "RWR");
        const vsg::dvec3 bRight = bodyToEcefVec(st, vsg::dvec3(0.0, 1.0, 0.0));
        for (const auto& e : frame_.esmRays) {
            if (e.observerSlot != ownSlot) continue;
            const vsg::dvec3 ray(e.rayDirEcef[0], e.rayDirEcef[1], e.rayDirEcef[2]);
            const double fx = vsg::dot(ray, fwd);
            const double fy = vsg::dot(ray, bRight);
            const double ang = std::atan2(fy, fx);
            const bool warn = (e.flags & ipc::kTactEsmWarning) != 0u;
            const float distR = rwrR * (warn ? 0.45f : 0.78f);
            const ImVec2 tp(rc.x + static_cast<float>(std::sin(ang)) * distR, rc.y - static_cast<float>(std::cos(ang)) * distR);
            dl->AddCircleFilled(tp, 4.0f * k, warn ? hudWarn : hud);
            if (warn) dl->AddText(ImVec2(rc.x - 24.0f * k, rc.y + rwrR + 6.0f * k), hudWarn, "LOCK WARN");
        }
    }
}

void TacticalOverlays::drawPipPanel() const {
    if (!pipActive_ || !pipDescriptorSet_) return;
    const VkDescriptorSet vkSet = pipDescriptorSet_->vk(deviceID_);
    if (vkSet == VK_NULL_HANDLE) return;

    const float k = ImGui::GetIO().FontGlobalScale;
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float margin = 14.0f * k;
    const float imgW = std::min(440.0f * k, display.x * 0.28f);
    const float aspect = static_cast<float>(pipHeight_) / static_cast<float>(std::max(1u, pipWidth_));
    const float imgH = imgW * aspect;

    ImGui::SetNextWindowPos(ImVec2(display.x - margin, margin), ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.04f, 0.07f, 0.10f, 0.88f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.24f, 0.82f, 0.56f, 0.85f));
    const ImGuiWindowFlags flags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
    if (ImGui::Begin(pipTitle_.empty() ? "PiP Camera  [I]" : pipTitle_.c_str(), nullptr, flags)) {
        const ImTextureID texId = static_cast<ImTextureID>(reinterpret_cast<std::uintptr_t>(vkSet));
        ImGui::Image(texId, ImVec2(imgW, imgH));
        if (!pipSubtitle_.empty()) ImGui::TextColored(ImVec4(0.55f, 0.96f, 0.72f, 1.0f), "%s", pipSubtitle_.c_str());
        int pm = controls_->pipMode.load(std::memory_order_relaxed);
        const char* pModes[] = {"Auto (Weapon/Target)", "Weapon Follow", "Tracked Target"};
        ImGui::SetNextItemWidth(imgW * 0.65f);
        if (ImGui::Combo("##pipmode", &pm, pModes, 3)) controls_->pipMode.store(pm, std::memory_order_relaxed);
        ImGui::SameLine();
        if (ImGui::SmallButton("Hide [I]")) controls_->pipEnabled.store(false, std::memory_order_relaxed);
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
}

void TacticalOverlays::record(vsg::CommandBuffer&) const {
    const std::uint32_t mask = controls_ ? controls_->tacticalOverlayMask.load(std::memory_order_relaxed) : 0u;
    const bool cockpit = (cameraMode_ == world::CameraController::Mode::Cockpit);
    const bool pip = pipActive_ && controls_ && controls_->pipEnabled.load(std::memory_order_relaxed);
    if (mask == 0u && !cockpit && !pip) {
        lastDrawMs_ = 0.0;
        return;
    }

    const auto t0 = std::chrono::steady_clock::now();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    if (hasFrame_ && mask != 0u) {
        if (mask & kOverlaySwaths)  drawSwaths(dl);
        if (mask & kOverlayJammer)  drawJammerLobes(dl);
        if (mask & kOverlayRadar)   drawRadarVolumes(dl);
        if (mask & kOverlayEsm)     drawEsmRays(dl);
        if (mask & kOverlayLinks)   drawLinkPulses(dl);
        if (mask & kOverlayWeapons) drawWeaponCues(dl);
    }
    if (cockpit) drawCockpitHud(dl);
    if (pip)     drawPipPanel();
    const auto t1 = std::chrono::steady_clock::now();
    lastDrawMs_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

} // namespace fsim::ui
