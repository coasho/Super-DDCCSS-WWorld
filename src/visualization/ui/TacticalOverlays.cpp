#include "visualization/ui/TacticalOverlays.h"

#include "visualization/vfx/effects/TacticalVolumes.h"

#include <vsgImGui/imgui.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace fsim::ui {

namespace {

constexpr double kNearClipViewZ = -2.0;

vsg::dvec3 bodyToEcefVec(const sim::VehicleState& s, const vsg::dvec3& b) noexcept {
    const double* R = s.rotationBodyToEcef;
    return vsg::dvec3(
        R[0] * b.x + R[1] * b.y + R[2] * b.z,
        R[3] * b.x + R[4] * b.y + R[5] * b.z,
        R[6] * b.x + R[7] * b.y + R[8] * b.z);
}

ImU32 sideAccent(world::TacticalVolumes::Side side) noexcept {
    if (side == world::TacticalVolumes::Side::Red)  return IM_COL32(255, 72, 66, 255);
    if (side == world::TacticalVolumes::Side::Blue) return IM_COL32(56, 188, 255, 255);
    return IM_COL32(155, 210, 235, 255);
}

} // namespace

TacticalOverlays::TacticalOverlays(std::shared_ptr<ViewerControls> controls)
    : controls_(std::move(controls)) {}

void TacticalOverlays::update(const ipc::TacticalOverlayFrameV1* frame,
                              Span<const sim::VehicleState> states,
                              const std::vector<unsigned char>& alive,
                              const std::vector<MonitorGui::VehicleMeta>& meta,
                              int selectedSlot,
                              const vsg::dmat4& viewMatrix,
                              const vsg::dmat4& projMatrix,
                              const vsg::dvec3& eyeEcef,
                              std::uint32_t screenWidth,
                              std::uint32_t screenHeight,
                              double simTime) {
    selectedSlot_ = selectedSlot;
    view_ = viewMatrix;
    proj_ = projMatrix;
    eye_ = eyeEcef;
    width_ = std::max(1u, screenWidth);
    height_ = std::max(1u, screenHeight);
    simTime_ = simTime;

    const std::uint32_t mask = controls_ ? controls_->tacticalOverlayMask.load(std::memory_order_relaxed) : 0u;
    if (mask == 0u || !frame) {
        hasFrame_ = false;
        states_.clear();
        alive_.clear();
        return;
    }

    frame_ = *frame;
    hasFrame_ = true;
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
    if (!std::isfinite(nx) || !std::isfinite(ny) || nx < -1.05 || nx > 1.05 || ny < -1.05 || ny > 1.05) return false;
    sx = static_cast<float>((nx * 0.5 + 0.5) * static_cast<double>(width_));
    sy = static_cast<float>((ny * 0.5 + 0.5) * static_cast<double>(height_));
    return true;
}

void TacticalOverlays::drawPill(void* drawList, float anchorX, float anchorY, const char* text,
                                std::uint32_t accentCol, std::vector<PlacedBox>& placed,
                                float scale, bool drawLeader) const {
    if (!text || !text[0]) return;
    auto* dl = static_cast<ImDrawList*>(drawList);
    const ImVec2 sz = ImGui::CalcTextSize(text);
    const float padX = 6.0f * scale;
    const float padY = 2.5f * scale;
    const float barW = 2.8f * scale;
    const float boxW = barW + padX * 2.0f + sz.x;
    const float boxH = padY * 2.0f + sz.y;

    float x0 = anchorX + 10.0f * scale;
    float y0 = anchorY - boxH * 0.5f;
    // Keep comfortably inside screen margins
    x0 = std::clamp(x0, 12.0f * scale, std::max(16.0f * scale, static_cast<float>(width_) - boxW - 12.0f * scale));
    y0 = std::clamp(y0, 12.0f * scale, std::max(16.0f * scale, static_cast<float>(height_) - boxH - 12.0f * scale));

    // Anti-overlap collision nudge so tactical callouts never pile on top of each other or aircraft tags
    const float gap = 4.0f * scale;
    for (int iter = 0; iter < 8; ++iter) {
        bool hit = false;
        for (const auto& b : placed) {
            if (x0 < b.x1 + gap && x0 + boxW + gap > b.x0 && y0 < b.y1 + gap && y0 + boxH + gap > b.y0) {
                y0 = b.y1 + gap;
                hit = true;
                break;
            }
        }
        if (!hit) break;
    }

    const ImVec2 lo(x0, y0);
    const ImVec2 hi(x0 + boxW, y0 + boxH);
    placed.push_back({lo.x, lo.y, hi.x, hi.y});

    if (drawLeader) {
        const ImVec2 elbow(lo.x, (lo.y + hi.y) * 0.5f);
        dl->AddLine(ImVec2(anchorX, anchorY), elbow, (accentCol & 0x00FFFFFFu) | 0x99000000u, 1.2f * scale);
        dl->AddCircleFilled(ImVec2(anchorX, anchorY), 2.6f * scale, accentCol);
    }

    // Subtle dark glassmorphic pill with thin colored tactical accent bar (zero garish solid blocks)
    dl->AddRectFilled(lo, hi, IM_COL32(8, 13, 22, 198), 3.0f * scale);
    dl->AddRect(lo, hi, IM_COL32(255, 255, 255, 34), 3.0f * scale, 0, 1.0f);
    dl->AddRectFilled(lo, ImVec2(lo.x + barW, hi.y), accentCol, 2.0f * scale);
    dl->AddText(ImVec2(lo.x + barW + padX, lo.y + padY), IM_COL32(236, 244, 252, 248), text);
}

void TacticalOverlays::record(vsg::CommandBuffer&) const {
    const std::uint32_t mask = controls_ ? controls_->tacticalOverlayMask.load(std::memory_order_relaxed) : 0u;
    if (mask == 0u || !hasFrame_) {
        lastDrawMs_ = 0.0;
        return;
    }

    const auto t0 = std::chrono::steady_clock::now();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const float k = std::max(ImGui::GetIO().FontGlobalScale, 0.75f);

    // Pre-reserve top-left MonitorGui, bottom-left vehicle list, and vehicle label positions
    // so tactical pills never overlap HUD panels or aircraft callsigns!
    std::vector<PlacedBox> placed;
    placed.reserve(36);
    placed.push_back({0.0f, 0.0f, 350.0f * k, 410.0f * k});
    placed.push_back({0.0f, std::max(0.0f, static_cast<float>(height_) - 260.0f * k), 490.0f * k, static_cast<float>(height_)});
    for (std::size_t i = 0; i < states_.size() && i < alive_.size(); ++i) {
        if (!alive_[i]) continue;
        float vx = 0.0f, vy = 0.0f;
        const vsg::dvec3 p(states_[i].positionEcef[0], states_[i].positionEcef[1], states_[i].positionEcef[2]);
        if (projectPoint(p, vx, vy)) {
            placed.push_back({vx - 6.0f * k, vy - 14.0f * k, vx + 96.0f * k, vy + 14.0f * k});
        }
    }

    auto sideOfSlot = [&](std::uint32_t slot) {
        if (slot < meta_.size()) return world::TacticalVolumes::inferSide(meta_[slot].name);
        return world::TacticalVolumes::Side::Neutral;
    };

    // 1. Radar Burn-Through & Active STT/TWS Markers
    if (mask & kOverlayRadar) {
        for (const auto& r : frame_.radars) {
            if (r.slot >= states_.size() || r.slot >= alive_.size() || !alive_[r.slot]) continue;
            const sim::VehicleState& st = states_[r.slot];
            const vsg::dvec3 origin(st.positionEcef[0], st.positionEcef[1], st.positionEcef[2]);
            const vsg::dvec3 bFwd = bodyToEcefVec(st, vsg::dvec3(1.0, 0.0, 0.0));
            const vsg::dvec3 bRight = bodyToEcefVec(st, vsg::dvec3(0.0, 1.0, 0.0));
            const vsg::dvec3 bUp = bodyToEcefVec(st, vsg::dvec3(0.0, 0.0, -1.0));
            const auto side = sideOfSlot(r.slot);
            const ImU32 accent = sideAccent(side);
            const bool jammed = (r.flags & ipc::kTactRadarJammed) != 0u;
            const bool stt = (r.flags & ipc::kTactRadarSttLock) != 0u;
            const bool illum = (r.flags & ipc::kTactRadarIlluminating) != 0u;

            // Place burn-through callout cleanly at the flank of the burn-through dome (never on top of aircraft!)
            if (jammed && r.burnThroughRangeM > 100.0f && r.burnThroughRangeM < r.detectionRangeM * 0.96f) {
                const double azFlank = static_cast<double>(r.azimuthCentreRad) + static_cast<double>(r.azimuthHalfWidthRad) * 0.72;
                const double elMid = static_cast<double>(r.elevationCentreRad);
                const vsg::dvec3 flankDir = bFwd * (std::cos(elMid) * std::cos(azFlank))
                                          + bRight * (std::cos(elMid) * std::sin(azFlank))
                                          + bUp * std::sin(elMid);
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(origin + flankDir * static_cast<double>(r.burnThroughRangeM), sx, sy)) {
                    char buf[64];
                    std::snprintf(buf, sizeof buf, "BT %.1f km  J/N +%.0f dB",
                                  static_cast<double>(r.burnThroughRangeM) * 1e-3,
                                  static_cast<double>(r.jammerRatioDb));
                    drawPill(dl, sx, sy, buf, IM_COL32(255, 214, 92, 255), placed, k, true);
                }
            }

            if ((r.flags & ipc::kTactRadarHasTarget) != 0u && (stt || illum || static_cast<int>(r.slot) == selectedSlot_)) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(r.targetEcef[0], r.targetEcef[1], r.targetEcef[2]), sx, sy)) {
                    const float rad = (stt || illum ? 10.0f : 7.0f) * k;
                    const ImVec2 pts[4] = {{sx, sy - rad}, {sx + rad, sy}, {sx, sy + rad}, {sx - rad, sy}};
                    dl->AddPolyline(pts, 4, accent, ImDrawFlags_Closed, 1.6f * k);
                    if (stt || illum) {
                        char buf[40];
                        std::snprintf(buf, sizeof buf, "%s %.1f km", illum ? "ILLUM" : "STT",
                                      static_cast<double>(r.targetRangeM) * 1e-3);
                        drawPill(dl, sx, sy, buf, accent, placed, k, false);
                    }
                }
            }
        }
    }

    // 2. EA Jammer Callout (placed cleanly at cone flank)
    if (mask & kOverlayJammer) {
        for (const auto& j : frame_.jammers) {
            const vsg::dvec3 origin(j.originEcef[0], j.originEcef[1], j.originEcef[2]);
            vsg::dvec3 dir(j.boresightEcef[0], j.boresightEcef[1], j.boresightEcef[2]);
            if (vsg::length(dir) < 1e-6) continue;
            dir = vsg::normalize(dir);
            const bool comms = (j.flags & ipc::kTactJamComms) != 0u;
            const double rng = std::clamp(static_cast<double>(j.rangeM), 14000.0, 32000.0);
            const vsg::dvec3 ref = std::abs(dir.z) < 0.9 ? vsg::dvec3(0.0, 0.0, 1.0) : vsg::dvec3(1.0, 0.0, 0.0);
            const vsg::dvec3 sideVec = vsg::normalize(vsg::cross(ref, dir));
            const double halfAng = std::clamp(static_cast<double>(j.halfAngleRad), 0.08, 0.85);
            const vsg::dvec3 tagPos = (j.flags & ipc::kTactJamAllRound)
                                          ? origin
                                          : (origin + dir * (rng * 0.62) + sideVec * (rng * 0.62 * std::tan(halfAng) * 0.82));
            float sx = 0.0f, sy = 0.0f;
            if (projectPoint(tagPos, sx, sy)) {
                char buf[56];
                std::snprintf(buf, sizeof buf, "%s %.0f dBW  %.1f GHz", comms ? "COMMS JAM" : "EA JAM",
                              static_cast<double>(j.erpDbw), static_cast<double>(j.frequencyGhz));
                drawPill(dl, sx, sy, buf, comms ? IM_COL32(255, 165, 42, 255) : IM_COL32(226, 72, 250, 255), placed, k, true);
            }
        }
    }

    // 3. ESM CRLB Geolocation Fix Callouts (only when 3-D geolocation has converged)
    if (mask & kOverlayEsm) {
        for (const auto& e : frame_.esmRays) {
            if ((e.flags & ipc::kTactEsmHasGeo) == 0u || !(e.crlbCep50M > 1.0f)) continue;
            float sx = 0.0f, sy = 0.0f;
            if (projectPoint(vsg::dvec3(e.geoPosEcef[0], e.geoPosEcef[1], e.geoPosEcef[2]), sx, sy)) {
                char buf[44];
                if (e.crlbCep50M >= 1000.0f) std::snprintf(buf, sizeof buf, "CRLB %.2f km", static_cast<double>(e.crlbCep50M) * 1e-3);
                else std::snprintf(buf, sizeof buf, "CRLB %.0f m", static_cast<double>(e.crlbCep50M));
                const ImU32 accent = (e.flags & ipc::kTactEsmWarning) ? IM_COL32(255, 78, 56, 255) : IM_COL32(255, 208, 56, 255);
                drawPill(dl, sx, sy, buf, accent, placed, k, true);
            }
        }
    }

    // 4. Selected Vehicle Weapon DLZ / LCOS / CCIP Reticle & Micro-Pill
    if (mask & kOverlayWeapons) {
        for (const auto& w : frame_.weapons) {
            if (w.slot >= states_.size() || w.slot >= alive_.size() || !alive_[w.slot]) continue;
            if (selectedSlot_ >= 0 && static_cast<int>(w.slot) != selectedSlot_) continue;

            if ((w.flags & ipc::kTactWeaponHasDlz) != 0u && w.rMaxM > 100.0f) {
                float sx = 0.0f, sy = 0.0f;
                const vsg::dvec3 orig(states_[w.slot].positionEcef[0], states_[w.slot].positionEcef[1], states_[w.slot].positionEcef[2]);
                const vsg::dvec3 tgt(w.targetEcef[0], w.targetEcef[1], w.targetEcef[2]);
                vsg::dvec3 dir = tgt - orig;
                if (vsg::length2(dir) > 100.0) dir = vsg::normalize(dir);
                else dir = bodyToEcefVec(states_[w.slot], vsg::dvec3(1.0, 0.0, 0.0));
                if (projectPoint(orig + dir * static_cast<double>(w.rNeM), sx, sy)) {
                    char buf[72];
                    const bool inNez = (w.flags & ipc::kTactWeaponInNez) != 0u;
                    std::snprintf(buf, sizeof buf, "%s  Rmax %.0f / Rne %.0f km%s",
                                  w.storeName[0] ? w.storeName : "AAM",
                                  static_cast<double>(w.rMaxM) * 1e-3,
                                  static_cast<double>(w.rNeM) * 1e-3,
                                  inNez ? "  [NEZ]" : "");
                    drawPill(dl, sx, sy, buf, inNez ? IM_COL32(36, 240, 130, 255) : IM_COL32(245, 190, 62, 255), placed, k, true);
                }
            }
            if ((w.flags & ipc::kTactWeaponHasLcos) != 0u) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(w.lcosPipperEcef[0], w.lcosPipperEcef[1], w.lcosPipperEcef[2]), sx, sy)) {
                    const ImU32 col = IM_COL32(82, 255, 148, 225);
                    dl->AddCircle(ImVec2(sx, sy), 9.0f * k, col, 24, 1.5f * k);
                    dl->AddCircleFilled(ImVec2(sx, sy), 2.0f * k, col);
                }
            }
            if ((w.flags & ipc::kTactWeaponHasCcip) != 0u) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(w.ccipImpactEcef[0], w.ccipImpactEcef[1], w.ccipImpactEcef[2]), sx, sy)) {
                    const ImU32 col = IM_COL32(255, 198, 52, 235);
                    dl->AddCircle(ImVec2(sx, sy), 9.0f * k, col, 20, 1.6f * k);
                    dl->AddLine(ImVec2(sx - 13.0f * k, sy), ImVec2(sx + 13.0f * k, sy), col, 1.3f * k);
                    dl->AddLine(ImVec2(sx, sy - 13.0f * k), ImVec2(sx, sy + 13.0f * k), col, 1.3f * k);
                }
            }
        }
    }

    // 5. Link 16 / C2 / PNT Minimal Callouts
    if (mask & kOverlayLinks) {
        for (const auto& l : frame_.links) {
            const auto kind = static_cast<ipc::TacticalLinkKindV1>(l.kind);
            if (kind == ipc::TacticalLinkKindV1::C2Consent) {
                const vsg::dvec3 mid = (vsg::dvec3(l.fromEcef[0], l.fromEcef[1], l.fromEcef[2]) +
                                        vsg::dvec3(l.toEcef[0], l.toEcef[1], l.toEcef[2])) * 0.5;
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(mid, sx, sy)) drawPill(dl, sx, sy, "C2 CONSENT", IM_COL32(255, 208, 56, 255), placed, k, false);
            } else if (kind == ipc::TacticalLinkKindV1::PntGnssDrift && (l.flags & ipc::kTactLinkJammed) != 0u) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(l.fromEcef[0], l.fromEcef[1], l.fromEcef[2]), sx, sy)) {
                    char buf[40];
                    std::snprintf(buf, sizeof buf, "GNSS JAM %.0f m", static_cast<double>(l.param));
                    drawPill(dl, sx, sy, buf, IM_COL32(255, 72, 62, 255), placed, k, false);
                }
            }
        }
    }

    // 6. SAR / Weather Swath Minimal Callouts
    if (mask & kOverlaySwaths) {
        for (const auto& s : frame_.swaths) {
            float sx = 0.0f, sy = 0.0f;
            if (projectPoint(vsg::dvec3(s.centerEcef[0], s.centerEcef[1], s.centerEcef[2]), sx, sy)) {
                char buf[48];
                if (s.kind == static_cast<std::uint8_t>(ipc::TacticalSwathKindV1::WeatherCell))
                    std::snprintf(buf, sizeof buf, "WX %.0f dBZ", static_cast<double>(s.metricDb));
                else
                    std::snprintf(buf, sizeof buf, "SAR %.1f m  NESZ %.0f dB", static_cast<double>(s.resolutionM), static_cast<double>(s.metricDb));
                drawPill(dl, sx, sy, buf, IM_COL32(42, 218, 240, 255), placed, k, false);
            }
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    lastDrawMs_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

} // namespace fsim::ui
