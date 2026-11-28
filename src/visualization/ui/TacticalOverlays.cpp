#include "visualization/ui/TacticalOverlays.h"

#include "visualization/ui/Colour.h"
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
    if (side == world::TacticalVolumes::Side::Red)  return colour(245, 76, 72, 255);
    if (side == world::TacticalVolumes::Side::Blue) return colour(56, 184, 255, 255);
    return colour(145, 206, 232, 255);
}

bool jammerIlluminatesPoint(const ipc::TacticalJammerLobeWireV1& j, const vsg::dvec3& targetEcef) noexcept {
    const vsg::dvec3 origin(j.originEcef[0], j.originEcef[1], j.originEcef[2]);
    const vsg::dvec3 delta = targetEcef - origin;
    const double dist = vsg::length(delta);
    if (dist < 1.0) return false;
    const double maxRange = std::max(static_cast<double>(j.rangeM), 15000.0) * 1.15;
    if (dist > maxRange) return false;
    if ((j.flags & ipc::kTactJamAllRound) != 0u) return true;
    vsg::dvec3 dir(j.boresightEcef[0], j.boresightEcef[1], j.boresightEcef[2]);
    const double dn = vsg::length(dir);
    if (dn < 1e-6) return false;
    dir /= dn;
    const double halfAng = std::clamp(static_cast<double>(j.halfAngleRad), 0.08, 0.85) + 0.14;
    return vsg::dot(dir, delta / dist) >= std::cos(halfAng);
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
    if (mask == 0u || !frame || selectedSlot < 0) {
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
    if (!std::isfinite(nx) || !std::isfinite(ny) || nx < -1.04 || nx > 1.04 || ny < -1.04 || ny > 1.04) return false;
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
    const float padX = 7.0f * scale;
    const float padY = 3.0f * scale;
    const float barW = 3.0f * scale;
    const float boxW = barW + padX * 2.0f + sz.x;
    const float boxH = padY * 2.0f + sz.y;

    float x0 = anchorX + 12.0f * scale;
    float y0 = anchorY - boxH * 0.5f;
    x0 = std::clamp(x0, 12.0f * scale, std::max(16.0f * scale, static_cast<float>(width_) - boxW - 12.0f * scale));
    y0 = std::clamp(y0, 12.0f * scale, std::max(16.0f * scale, static_cast<float>(height_) - boxH - 12.0f * scale));

    const float gap = 5.0f * scale;
    for (int iter = 0; iter < 10; ++iter) {
        bool hit = false;
        for (const auto& b : placed) {
            if (x0 < b.x1 + gap && x0 + boxW + gap > b.x0 && y0 < b.y1 + gap && y0 + boxH + gap > b.y0) {
                if (x0 + boxW > static_cast<float>(width_) - 420.0f * scale && b.x0 > static_cast<float>(width_) * 0.55f) {
                    x0 = std::max(12.0f * scale, b.x0 - boxW - gap);
                } else {
                    y0 = b.y1 + gap;
                }
                hit = true;
                break;
            }
        }
        if (!hit) break;
    }
    x0 = std::clamp(x0, 12.0f * scale, std::max(16.0f * scale, static_cast<float>(width_) - boxW - 12.0f * scale));
    y0 = std::clamp(y0, 12.0f * scale, std::max(16.0f * scale, static_cast<float>(height_) - boxH - 12.0f * scale));

    const ImVec2 lo(x0, y0);
    const ImVec2 hi(x0 + boxW, y0 + boxH);
    placed.push_back({lo.x, lo.y, hi.x, hi.y});

    if (drawLeader) {
        const float edgeX = (anchorX < lo.x) ? lo.x : hi.x;
        const ImVec2 elbow(edgeX, (lo.y + hi.y) * 0.5f);
        dl->AddLine(ImVec2(anchorX, anchorY), elbow, colour(15, 22, 34, 180), 2.4f * scale);
        dl->AddLine(ImVec2(anchorX, anchorY), elbow, accentCol, 1.1f * scale);
        dl->AddCircleFilled(ImVec2(anchorX, anchorY), 2.8f * scale, accentCol);
    }

    // True dark slate tactical pill (properly sRGB->linear aware via ui::colour)
    dl->AddRectFilled(lo, hi, colour(12, 18, 27, 226), 4.0f * scale);
    dl->AddRect(lo, hi, colour(86, 114, 146, 118), 4.0f * scale, 0, 1.0f);
    dl->AddRectFilled(lo, ImVec2(lo.x + barW, hi.y), accentCol, 2.0f * scale);
    dl->AddText(ImVec2(lo.x + barW + padX, lo.y + padY), colour(238, 244, 250, 255), text);
}

void TacticalOverlays::record(vsg::CommandBuffer&) const {
    const std::uint32_t mask = controls_ ? controls_->tacticalOverlayMask.load(std::memory_order_relaxed) : 0u;
    if (mask == 0u || !hasFrame_ || selectedSlot_ < 0 ||
        static_cast<std::size_t>(selectedSlot_) >= states_.size() ||
        static_cast<std::size_t>(selectedSlot_) >= alive_.size() ||
        !alive_[static_cast<std::size_t>(selectedSlot_)]) {
        lastDrawMs_ = 0.0;
        return;
    }

    const auto t0 = std::chrono::steady_clock::now();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const float k = uiScale();

    // Pre-reserve MonitorGui (top-left), vehicle table (bottom-left), Radar UI (top-right),
    // and visible aircraft labels so tactical pills never overlap any panel or callsign.
    std::vector<PlacedBox> placed;
    placed.reserve(32);
    placed.push_back({0.0f, 0.0f, std::min(360.0f * k, static_cast<float>(width_) * 0.36f), 460.0f * k});
    placed.push_back({0.0f, std::max(0.0f, static_cast<float>(height_) - 250.0f * k),
                      std::min(510.0f * k, static_cast<float>(width_) * 0.48f), static_cast<float>(height_)});
    placed.push_back({std::max(0.0f, static_cast<float>(width_) - std::min(410.0f * k, static_cast<float>(width_) * 0.32f)),
                      0.0f, static_cast<float>(width_), std::min(620.0f * k, static_cast<float>(height_) * 0.62f)});
    for (std::size_t i = 0; i < states_.size() && i < alive_.size(); ++i) {
        if (!alive_[i]) continue;
        float vx = 0.0f, vy = 0.0f;
        const vsg::dvec3 p(states_[i].positionEcef[0], states_[i].positionEcef[1], states_[i].positionEcef[2]);
        if (projectPoint(p, vx, vy)) {
            placed.push_back({vx - 6.0f * k, vy - 16.0f * k, vx + 110.0f * k, vy + 16.0f * k});
        }
    }

    auto sideOfSlot = [&](std::uint32_t slot) {
        if (slot < meta_.size()) return world::TacticalVolumes::inferSide(meta_[slot].name);
        return world::TacticalVolumes::Side::Neutral;
    };

    bool focusedRadarJammed = false;

    // 1. Focused Vehicle Radar Burn-Through & Track Callouts
    if (mask & kOverlayRadar) {
        for (const auto& r : frame_.radars) {
            if (static_cast<int>(r.slot) != selectedSlot_) continue;
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
            if (jammed) focusedRadarJammed = true;

            if (jammed && r.burnThroughRangeM > 100.0f && r.burnThroughRangeM < r.detectionRangeM * 0.96f) {
                const double azFlank = static_cast<double>(r.azimuthCentreRad) + static_cast<double>(r.azimuthHalfWidthRad) * 0.68;
                const double elMid = static_cast<double>(r.elevationCentreRad);
                const vsg::dvec3 flankDir = bFwd * (std::cos(elMid) * std::cos(azFlank))
                                          + bRight * (std::cos(elMid) * std::sin(azFlank))
                                          + bUp * std::sin(elMid);
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(origin + flankDir * static_cast<double>(r.burnThroughRangeM), sx, sy)) {
                    char buf[72];
                    std::snprintf(buf, sizeof buf, "烧穿 %.1f km · 压制 +%.0f dB",
                                  static_cast<double>(r.burnThroughRangeM) * 1e-3,
                                  static_cast<double>(r.jammerRatioDb));
                    drawPill(dl, sx, sy, buf, colour(255, 206, 52, 255), placed, k, true);
                }
            }

            if ((r.flags & ipc::kTactRadarHasTarget) != 0u) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(r.targetEcef[0], r.targetEcef[1], r.targetEcef[2]), sx, sy)) {
                    const float rad = (stt || illum ? 10.0f : 7.0f) * k;
                    const ImVec2 pts[4] = {{sx, sy - rad}, {sx + rad, sy}, {sx, sy + rad}, {sx - rad, sy}};
                    dl->AddPolyline(pts, 4, colour(10, 16, 24, 210), ImDrawFlags_Closed, 3.0f * k);
                    dl->AddPolyline(pts, 4, accent, ImDrawFlags_Closed, 1.5f * k);
                    if (stt || illum) {
                        char buf[56];
                        std::snprintf(buf, sizeof buf, "%s %.1f km", illum ? "照射引导" : "单目标锁定",
                                      static_cast<double>(r.targetRangeM) * 1e-3);
                        drawPill(dl, sx, sy, buf, accent, placed, k, false);
                    }
                }
            }
        }
    }

    // 2. EA Jammer Callout (focused jammer OR incoming jammer actively attacking focused radar)
    if (mask & kOverlayJammer) {
        const sim::VehicleState& selSt = states_[static_cast<std::size_t>(selectedSlot_)];
        const vsg::dvec3 selPos(selSt.positionEcef[0], selSt.positionEcef[1], selSt.positionEcef[2]);
        for (const auto& j : frame_.jammers) {
            const bool isOwner = (static_cast<int>(j.slot) == selectedSlot_);
            const bool attackingFocused = !isOwner && focusedRadarJammed && jammerIlluminatesPoint(j, selPos);
            if (!isOwner && !attackingFocused) continue;

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
                                          : (origin + dir * (rng * 0.56) + sideVec * (rng * 0.56 * std::tan(halfAng) * 0.85));
            float sx = 0.0f, sy = 0.0f;
            if (projectPoint(tagPos, sx, sy)) {
                char buf[72];
                std::snprintf(buf, sizeof buf, "%s %.0f dBW · %.1f GHz", comms ? "通信干扰" : "雷达干扰",
                              static_cast<double>(j.erpDbw), static_cast<double>(j.frequencyGhz));
                drawPill(dl, sx, sy, buf, comms ? colour(245, 152, 42, 255) : colour(192, 92, 248, 255), placed, k, true);
            }
        }
    }

    // 3. Focused Vehicle ESM CRLB Geolocation Callout
    if (mask & kOverlayEsm) {
        for (const auto& e : frame_.esmRays) {
            if (static_cast<int>(e.observerSlot) != selectedSlot_) continue;
            if ((e.flags & ipc::kTactEsmHasGeo) == 0u || !(e.crlbCep50M > 1.0f)) continue;
            float sx = 0.0f, sy = 0.0f;
            if (projectPoint(vsg::dvec3(e.geoPosEcef[0], e.geoPosEcef[1], e.geoPosEcef[2]), sx, sy)) {
                char buf[56];
                if (e.crlbCep50M >= 1000.0f) std::snprintf(buf, sizeof buf, "定位误差 %.2f km", static_cast<double>(e.crlbCep50M) * 1e-3);
                else std::snprintf(buf, sizeof buf, "定位误差 %.0f m", static_cast<double>(e.crlbCep50M));
                const ImU32 accent = (e.flags & ipc::kTactEsmWarning) ? colour(245, 78, 56, 255) : colour(250, 204, 56, 255);
                drawPill(dl, sx, sy, buf, accent, placed, k, true);
            }
        }
    }

    // 4. Focused Vehicle Weapon DLZ / LCOS / CCIP Reticle & Micro-Pill
    if (mask & kOverlayWeapons) {
        for (const auto& w : frame_.weapons) {
            if (static_cast<int>(w.slot) != selectedSlot_) continue;

            if ((w.flags & ipc::kTactWeaponHasDlz) != 0u && w.rMaxM > 100.0f) {
                float sx = 0.0f, sy = 0.0f;
                const vsg::dvec3 orig(states_[w.slot].positionEcef[0], states_[w.slot].positionEcef[1], states_[w.slot].positionEcef[2]);
                const vsg::dvec3 tgt(w.targetEcef[0], w.targetEcef[1], w.targetEcef[2]);
                vsg::dvec3 dir = tgt - orig;
                if (vsg::length2(dir) > 100.0) dir = vsg::normalize(dir);
                else dir = bodyToEcefVec(states_[w.slot], vsg::dvec3(1.0, 0.0, 0.0));
                if (projectPoint(orig + dir * static_cast<double>(w.rNeM), sx, sy)) {
                    char buf[88];
                    const bool inNez = (w.flags & ipc::kTactWeaponInNez) != 0u;
                    std::snprintf(buf, sizeof buf, "%s  射程 %.0f / 不可逃逸 %.0f km%s",
                                  w.storeName[0] ? w.storeName : "空空导弹",
                                  static_cast<double>(w.rMaxM) * 1e-3,
                                  static_cast<double>(w.rNeM) * 1e-3,
                                  inNez ? " [可发射]" : "");
                    drawPill(dl, sx, sy, buf, inNez ? colour(42, 224, 126, 255) : colour(242, 188, 60, 255), placed, k, true);
                }
            }
            if ((w.flags & ipc::kTactWeaponHasLcos) != 0u) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(w.lcosPipperEcef[0], w.lcosPipperEcef[1], w.lcosPipperEcef[2]), sx, sy)) {
                    const ImU32 col = colour(80, 240, 146, 235);
                    dl->AddCircle(ImVec2(sx, sy), 9.0f * k, colour(10, 16, 24, 200), 24, 2.8f * k);
                    dl->AddCircle(ImVec2(sx, sy), 9.0f * k, col, 24, 1.5f * k);
                    dl->AddCircleFilled(ImVec2(sx, sy), 2.0f * k, col);
                }
            }
            if ((w.flags & ipc::kTactWeaponHasCcip) != 0u) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(w.ccipImpactEcef[0], w.ccipImpactEcef[1], w.ccipImpactEcef[2]), sx, sy)) {
                    const ImU32 col = colour(255, 198, 52, 240);
                    dl->AddCircle(ImVec2(sx, sy), 9.0f * k, colour(10, 16, 24, 200), 20, 2.8f * k);
                    dl->AddCircle(ImVec2(sx, sy), 9.0f * k, col, 20, 1.5f * k);
                    dl->AddLine(ImVec2(sx - 13.0f * k, sy), ImVec2(sx + 13.0f * k, sy), col, 1.3f * k);
                    dl->AddLine(ImVec2(sx, sy - 13.0f * k), ImVec2(sx, sy + 13.0f * k), col, 1.3f * k);
                }
            }
        }
    }

    // 5. Focused Vehicle Link 16 / C2 / PNT Callouts
    if (mask & kOverlayLinks) {
        for (const auto& l : frame_.links) {
            const auto kind = static_cast<ipc::TacticalLinkKindV1>(l.kind);
            const bool involvesSelected =
                (static_cast<int>(l.fromSlot) == selectedSlot_) ||
                ((kind == ipc::TacticalLinkKindV1::Link16Dms || kind == ipc::TacticalLinkKindV1::C2Consent ||
                  kind == ipc::TacticalLinkKindV1::Mode5IffReply) &&
                 static_cast<int>(l.toSlot) == selectedSlot_);
            if (!involvesSelected) continue;

            if (kind == ipc::TacticalLinkKindV1::C2Consent) {
                const vsg::dvec3 mid = (vsg::dvec3(l.fromEcef[0], l.fromEcef[1], l.fromEcef[2]) +
                                        vsg::dvec3(l.toEcef[0], l.toEcef[1], l.toEcef[2])) * 0.5;
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(mid, sx, sy)) drawPill(dl, sx, sy, "C2 授权", colour(255, 206, 56, 255), placed, k, false);
            } else if (kind == ipc::TacticalLinkKindV1::PntGnssDrift && (l.flags & ipc::kTactLinkJammed) != 0u) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(l.fromEcef[0], l.fromEcef[1], l.fromEcef[2]), sx, sy)) {
                    char buf[56];
                    std::snprintf(buf, sizeof buf, "导航干扰 %.0f m", static_cast<double>(l.param));
                    drawPill(dl, sx, sy, buf, colour(245, 74, 62, 255), placed, k, false);
                }
            }
        }
    }

    // 6. Focused Vehicle SAR / Weather Swath Callouts
    if (mask & kOverlaySwaths) {
        for (const auto& s : frame_.swaths) {
            if (static_cast<int>(s.slot) != selectedSlot_) continue;
            float sx = 0.0f, sy = 0.0f;
            if (projectPoint(vsg::dvec3(s.centerEcef[0], s.centerEcef[1], s.centerEcef[2]), sx, sy)) {
                char buf[64];
                if (s.kind == static_cast<std::uint8_t>(ipc::TacticalSwathKindV1::WeatherCell))
                    std::snprintf(buf, sizeof buf, "气象 %.0f dBZ", static_cast<double>(s.metricDb));
                else
                    std::snprintf(buf, sizeof buf, "SAR %.1f m · 噪底 %.0f dB", static_cast<double>(s.resolutionM), static_cast<double>(s.metricDb));
                drawPill(dl, sx, sy, buf, colour(56, 208, 236, 255), placed, k, false);
            }
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    lastDrawMs_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

} // namespace fsim::ui
