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

vsg::dvec3 localUpVec(const vsg::dvec3& p) noexcept {
    const double len = vsg::length(p);
    return len > 1.0 ? p / len : vsg::dvec3(0.0, 0.0, 1.0);
}

void localEnuVec(const vsg::dvec3& pos, vsg::dvec3& east, vsg::dvec3& north, vsg::dvec3& up) noexcept {
    up = localUpVec(pos);
    east = vsg::cross(vsg::dvec3(0.0, 0.0, 1.0), up);
    if (vsg::length(east) < 1e-6) east = vsg::dvec3(1.0, 0.0, 0.0);
    east = vsg::normalize(east);
    north = vsg::cross(up, east);
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

bool TacticalOverlays::pickAdaptiveAnchor(const std::vector<vsg::dvec3>& candidates,
                                          const std::vector<PlacedBox>& reservedPanels,
                                          float scale,
                                          float& outX, float& outY) const noexcept {
    struct Proj {
        float x = 0.0f, y = 0.0f;
        bool inSafe = false;
        float penalty = 0.0f;
    };
    std::vector<Proj> visible;
    visible.reserve(candidates.size());
    const float padX = 80.0f * scale;
    const float padY = 42.0f * scale;
    const float w = static_cast<float>(width_);
    const float h = static_cast<float>(height_);

    for (const auto& pt : candidates) {
        float sx = 0.0f, sy = 0.0f;
        if (!projectPoint(pt, sx, sy)) continue;
        if (sx < 8.0f || sx > w - 8.0f || sy < 8.0f || sy > h - 8.0f) continue;

        float pen = 0.0f;
        // Prefer points away from extreme screen margins so the pill fits comfortably inside the zone.
        if (sx < padX) pen += (padX - sx) * 2.0f;
        if (sx > w - padX) pen += (sx - (w - padX)) * 2.0f;
        if (sy < padY) pen += (padY - sy) * 2.0f;
        if (sy > h - padY) pen += (sy - (h - padY)) * 2.0f;

        bool insidePanel = false;
        for (const auto& b : reservedPanels) {
            if (sx >= b.x0 - 18.0f * scale && sx <= b.x1 + 18.0f * scale &&
                sy >= b.y0 - 14.0f * scale && sy <= b.y1 + 14.0f * scale) {
                insidePanel = true;
                pen += 2000.0f;
                break;
            }
        }
        visible.push_back({sx, sy, !insidePanel && pen < 1.0f, pen});
    }
    if (visible.empty()) return false;

    // Compute visual centroid of all unobstructed candidates (or all visible candidates if none unobstructed)
    float cx = 0.0f, cy = 0.0f;
    int count = 0;
    for (const auto& p : visible) {
        if (p.inSafe) { cx += p.x; cy += p.y; ++count; }
    }
    if (count == 0) {
        for (const auto& p : visible) { cx += p.x; cy += p.y; ++count; }
    }
    cx /= static_cast<float>(count);
    cy /= static_cast<float>(count);

    // Choose the visible point closest to the visible centroid with minimal panel/edge penalty.
    std::size_t bestIdx = 0;
    float bestScore = 1e30f;
    for (std::size_t i = 0; i < visible.size(); ++i) {
        const float d = std::hypot(visible[i].x - cx, visible[i].y - cy);
        const float score = visible[i].penalty + d;
        if (score < bestScore) {
            bestScore = score;
            bestIdx = i;
        }
    }
    outX = visible[bestIdx].x;
    outY = visible[bestIdx].y;
    return true;
}

void TacticalOverlays::drawPill(void* drawList, float anchorX, float anchorY, const char* text,
                                ZoneColor zc, std::vector<PlacedBox>& placed,
                                float scale, bool drawLeader, bool centered) const {
    if (!text || !text[0]) return;
    auto* dl = static_cast<ImDrawList*>(drawList);
    const ImVec2 sz = ImGui::CalcTextSize(text);
    const float padX = 7.5f * scale;
    const float padY = 3.0f * scale;
    const float barW = 3.2f * scale;
    const float boxW = barW + padX * 2.0f + sz.x;
    const float boxH = padY * 2.0f + sz.y;

    float x0 = centered ? (anchorX - boxW * 0.5f) : (anchorX + 12.0f * scale);
    float y0 = anchorY - boxH * 0.5f;
    x0 = std::clamp(x0, 12.0f * scale, std::max(16.0f * scale, static_cast<float>(width_) - boxW - 12.0f * scale));
    y0 = std::clamp(y0, 12.0f * scale, std::max(16.0f * scale, static_cast<float>(height_) - boxH - 12.0f * scale));

    const float gap = 5.0f * scale;
    for (int iter = 0; iter < 12; ++iter) {
        bool hit = false;
        for (const auto& b : placed) {
            if (x0 < b.x1 + gap && x0 + boxW + gap > b.x0 && y0 < b.y1 + gap && y0 + boxH + gap > b.y0) {
                if (x0 + boxW > static_cast<float>(width_) - 430.0f * scale && b.x0 > static_cast<float>(width_) * 0.55f) {
                    x0 = std::max(12.0f * scale, b.x0 - boxW - gap);
                } else if (y0 + boxH + gap + boxH > static_cast<float>(height_) - 12.0f * scale) {
                    y0 = std::max(12.0f * scale, b.y0 - boxH - gap);
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

    // Zone-matched yet high-contrast palette:
    // - Deep tinted dark charcoal pill backing (separates cleanly from the translucent zone behind it)
    // - Crisp vivid zone-colored rim & left accent bar
    // - Bright pale-tinted text in the same hue family for immediate region association
    const int bgR = std::clamp(11 + (zc.r * 12) / 100, 8, 52);
    const int bgG = std::clamp(14 + (zc.g * 12) / 100, 10, 56);
    const int bgB = std::clamp(20 + (zc.b * 12) / 100, 14, 64);
    const int txtR = std::clamp((252 * 68 + zc.r * 32) / 100, 185, 255);
    const int txtG = std::clamp((252 * 68 + zc.g * 32) / 100, 185, 255);
    const int txtB = std::clamp((252 * 68 + zc.b * 32) / 100, 185, 255);

    const ImU32 accentCol = colour(zc.r, zc.g, zc.b, 252);
    const ImU32 rimCol    = colour(zc.r, zc.g, zc.b, 178);
    const ImU32 bgCol     = colour(bgR, bgG, bgB, 234);
    const ImU32 textCol   = colour(txtR, txtG, txtB, 255);

    if (drawLeader) {
        const float edgeX = (anchorX < lo.x) ? lo.x : (anchorX > hi.x ? hi.x : (lo.x + hi.x) * 0.5f);
        const float edgeY = (anchorY < lo.y) ? lo.y : (anchorY > hi.y ? hi.y : (lo.y + hi.y) * 0.5f);
        if (std::hypot(anchorX - edgeX, anchorY - edgeY) > 6.0f * scale) {
            dl->AddLine(ImVec2(anchorX, anchorY), ImVec2(edgeX, edgeY), colour(8, 12, 18, 200), 2.6f * scale);
            dl->AddLine(ImVec2(anchorX, anchorY), ImVec2(edgeX, edgeY), accentCol, 1.2f * scale);
            dl->AddCircleFilled(ImVec2(anchorX, anchorY), 2.8f * scale, accentCol);
        }
    }

    dl->AddRectFilled(lo, hi, bgCol, 4.0f * scale);
    dl->AddRect(lo, hi, rimCol, 4.0f * scale, 0, 1.25f);
    dl->AddRectFilled(lo, ImVec2(lo.x + barW, hi.y), accentCol, 2.0f * scale);
    dl->AddText(ImVec2(lo.x + barW + padX, lo.y + padY), textCol, text);
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

    // Reserve HUD panels & aircraft callsigns so adaptive zone tags glide into clear open sky.
    std::vector<PlacedBox> panelBoxes;
    panelBoxes.reserve(4);
    panelBoxes.push_back({0.0f, 0.0f, std::min(365.0f * k, static_cast<float>(width_) * 0.36f), 470.0f * k});
    panelBoxes.push_back({0.0f, std::max(0.0f, static_cast<float>(height_) - 255.0f * k),
                          std::min(510.0f * k, static_cast<float>(width_) * 0.48f), static_cast<float>(height_)});
    panelBoxes.push_back({std::max(0.0f, static_cast<float>(width_) - std::min(415.0f * k, static_cast<float>(width_) * 0.32f)),
                          0.0f, static_cast<float>(width_), std::min(660.0f * k, static_cast<float>(height_) * 0.66f)});

    std::vector<PlacedBox> placed = panelBoxes;
    placed.reserve(40);
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
    auto sideZoneColor = [](world::TacticalVolumes::Side side) -> ZoneColor {
        if (side == world::TacticalVolumes::Side::Red)  return {248, 82, 82};
        if (side == world::TacticalVolumes::Side::Blue) return {52, 182, 255};
        return {140, 206, 232};
    };

    bool focusedRadarJammed = false;

    // 1. Focused Vehicle Radar Zones (Active Sector + Burn-Through Boundary + Suppressed Blind Zone + Target Lock)
    if (mask & kOverlayRadar) {
        for (const auto& r : frame_.radars) {
            if (static_cast<int>(r.slot) != selectedSlot_) continue;
            const sim::VehicleState& st = states_[r.slot];
            const vsg::dvec3 origin(st.positionEcef[0], st.positionEcef[1], st.positionEcef[2]);
            const vsg::dvec3 bFwd = bodyToEcefVec(st, vsg::dvec3(1.0, 0.0, 0.0));
            const vsg::dvec3 bRight = bodyToEcefVec(st, vsg::dvec3(0.0, 1.0, 0.0));
            const vsg::dvec3 bUp = bodyToEcefVec(st, vsg::dvec3(0.0, 0.0, -1.0));
            const auto side = sideOfSlot(r.slot);
            const ZoneColor radarCol = sideZoneColor(side);
            const bool roto = (r.flags & ipc::kTactRadarRotodome) != 0u;
            const bool jammed = (r.flags & ipc::kTactRadarJammed) != 0u;
            const bool stt = (r.flags & ipc::kTactRadarSttLock) != 0u;
            const bool illum = (r.flags & ipc::kTactRadarIlluminating) != 0u;
            if (jammed) focusedRadarJammed = true;

            const double rawRange = static_cast<double>(r.detectionRangeM > 100.0f ? r.detectionRangeM : r.rangeScaleM);
            const bool hasBurn = jammed && r.burnThroughRangeM > 100.0f && r.burnThroughRangeM < static_cast<float>(rawRange) * 0.96f;
            const double nomRange = hasBurn
                                        ? std::clamp(std::max(static_cast<double>(r.burnThroughRangeM) * 2.0,
                                                              static_cast<double>(r.burnThroughRangeM) + 9500.0),
                                                     14000.0, std::min(rawRange, 42000.0))
                                        : std::clamp(rawRange, 6000.0, 42000.0);
            const double azC = static_cast<double>(r.azimuthCentreRad);
            const double azH = std::clamp(static_cast<double>(r.azimuthHalfWidthRad), 0.08, 1.25);
            const double elC = static_cast<double>(r.elevationCentreRad);
            auto dirAt = [&](double az) {
                return bFwd * (std::cos(elC) * std::cos(az)) + bRight * (std::cos(elC) * std::sin(az)) + bUp * std::sin(elC);
            };

            // (a) Active Radar Zone Label (adaptive inside the visible active sector!)
            std::vector<vsg::dvec3> activeCands;
            const double activeOuter = hasBurn ? static_cast<double>(r.burnThroughRangeM) : nomRange;
            if (roto) {
                vsg::dvec3 e, n, u;
                localEnuVec(origin, e, n, u);
                for (int i = 0; i < 6; ++i) {
                    const double a = i * (3.141592653589793 / 3.0);
                    activeCands.push_back(origin + (e * std::cos(a) + n * std::sin(a)) * (activeOuter * 0.62));
                }
            } else {
                for (double rf : {0.58, 0.78, 0.42}) {
                    for (double uf : {-0.58, 0.58, 0.0}) {
                        activeCands.push_back(origin + dirAt(azC + azH * uf) * (activeOuter * rf));
                    }
                }
            }
            float ax = 0.0f, ay = 0.0f;
            if (pickAdaptiveAnchor(activeCands, panelBoxes, k, ax, ay)) {
                char buf[72];
                if (hasBurn) {
                    std::snprintf(buf, sizeof buf, "雷达烧穿有效区 %.1f km", static_cast<double>(r.burnThroughRangeM) * 1e-3);
                } else {
                    const char* sidePrefix = (side == world::TacticalVolumes::Side::Red) ? "红方雷达搜索区"
                                           : (side == world::TacticalVolumes::Side::Blue) ? "蓝方雷达搜索区"
                                                                                          : "雷达搜索区";
                    std::snprintf(buf, sizeof buf, "%s %.0f km", sidePrefix, rawRange * 1e-3);
                }
                drawPill(dl, ax, ay, buf, radarCol, placed, k, false, true);
            }

            // (b) Burn-Through Arc & Suppressed Outer Blind Zone Labels when jammed
            if (hasBurn) {
                std::vector<vsg::dvec3> btCands;
                std::vector<vsg::dvec3> blindCands;
                const double btR = static_cast<double>(r.burnThroughRangeM);
                const double blindR = btR + (nomRange - btR) * 0.52;
                if (!roto) {
                    for (double uf : {0.72, -0.72, 0.35, -0.35, 0.0}) {
                        btCands.push_back(origin + dirAt(azC + azH * uf) * btR);
                        blindCands.push_back(origin + dirAt(azC + azH * uf) * blindR);
                    }
                } else {
                    vsg::dvec3 e, n, u;
                    localEnuVec(origin, e, n, u);
                    for (int i = 0; i < 6; ++i) {
                        const double a = i * (3.141592653589793 / 3.0);
                        btCands.push_back(origin + (e * std::cos(a) + n * std::sin(a)) * btR);
                        blindCands.push_back(origin + (e * std::cos(a) + n * std::sin(a)) * blindR);
                    }
                }
                if (pickAdaptiveAnchor(btCands, panelBoxes, k, ax, ay)) {
                    char buf[72];
                    std::snprintf(buf, sizeof buf, "烧穿边界 %.1f km · 压制 +%.0f dB",
                                  btR * 1e-3, static_cast<double>(r.jammerRatioDb));
                    drawPill(dl, ax, ay, buf, {255, 206, 52}, placed, k, true, false);
                }
                if (nomRange - btR > 2000.0 && pickAdaptiveAnchor(blindCands, panelBoxes, k, ax, ay)) {
                    drawPill(dl, ax, ay, "干扰压制盲区", {166, 180, 204}, placed, k, false, true);
                }
            }

            // (c) Target Lock Marker
            if ((r.flags & ipc::kTactRadarHasTarget) != 0u) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(r.targetEcef[0], r.targetEcef[1], r.targetEcef[2]), sx, sy)) {
                    const float rad = (stt || illum ? 10.0f : 7.0f) * k;
                    const ImVec2 pts[4] = {{sx, sy - rad}, {sx + rad, sy}, {sx, sy + rad}, {sx - rad, sy}};
                    dl->AddPolyline(pts, 4, colour(10, 16, 24, 210), ImDrawFlags_Closed, 3.0f * k);
                    dl->AddPolyline(pts, 4, colour(radarCol.r, radarCol.g, radarCol.b, 255), ImDrawFlags_Closed, 1.5f * k);
                    if (stt || illum) {
                        char buf[56];
                        std::snprintf(buf, sizeof buf, "%s %.1f km", illum ? "照射引导" : "单目标锁定",
                                      static_cast<double>(r.targetRangeM) * 1e-3);
                        drawPill(dl, sx, sy, buf, radarCol, placed, k, false, false);
                    }
                }
            }
        }
    }

    // 2. EA Jammer Zone Label (camera-adaptive inside the visible portion of the jammer cone)
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
            const vsg::dvec3 upVec = vsg::normalize(vsg::cross(dir, sideVec));
            const double halfAng = std::clamp(static_cast<double>(j.halfAngleRad), 0.08, 0.85);

            std::vector<vsg::dvec3> cands;
            if ((j.flags & ipc::kTactJamAllRound) != 0u) {
                vsg::dvec3 e, n, u;
                localEnuVec(origin, e, n, u);
                const double rad = rng * 0.35;
                cands.push_back(origin + e * rad);
                cands.push_back(origin - e * rad);
                cands.push_back(origin + n * rad);
                cands.push_back(origin - n * rad);
            } else {
                for (double df : {0.42, 0.65, 0.28}) {
                    const double d = rng * df;
                    const double w = d * std::tan(halfAng) * 0.68;
                    cands.push_back(origin + dir * d + sideVec * w);
                    cands.push_back(origin + dir * d - sideVec * w);
                    cands.push_back(origin + dir * d + upVec * (w * 0.6));
                    cands.push_back(origin + dir * d);
                }
            }
            float sx = 0.0f, sy = 0.0f;
            if (pickAdaptiveAnchor(cands, panelBoxes, k, sx, sy)) {
                char buf[76];
                std::snprintf(buf, sizeof buf, "%s %.0f dBW · %.1f GHz", comms ? "通信干扰区" : "电子干扰区",
                              static_cast<double>(j.erpDbw), static_cast<double>(j.frequencyGhz));
                const ZoneColor jCol = comms ? ZoneColor{245, 152, 42} : ZoneColor{198, 96, 252};
                drawPill(dl, sx, sy, buf, jCol, placed, k, false, true);
            }
        }
    }

    // 3. Focused Vehicle ESM Bearing & CRLB Ellipsoid Zone Labels
    if (mask & kOverlayEsm) {
        bool labeledBearing = false;
        for (const auto& e : frame_.esmRays) {
            if (static_cast<int>(e.observerSlot) != selectedSlot_) continue;
            const bool warn = (e.flags & ipc::kTactEsmWarning) != 0u;
            const bool comint = (e.flags & ipc::kTactEsmComint) != 0u;
            const ZoneColor eCol = warn ? ZoneColor{248, 82, 62}
                                 : comint ? ZoneColor{52, 232, 164}
                                          : ZoneColor{252, 206, 56};

            if ((e.flags & ipc::kTactEsmHasGeo) != 0u && e.crlbCep50M > 1.0f) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(e.geoPosEcef[0], e.geoPosEcef[1], e.geoPosEcef[2]), sx, sy)) {
                    char buf[64];
                    if (e.crlbCep50M >= 1000.0f) std::snprintf(buf, sizeof buf, "定位误差椭球 %.2f km", static_cast<double>(e.crlbCep50M) * 1e-3);
                    else std::snprintf(buf, sizeof buf, "定位误差椭球 %.0f m", static_cast<double>(e.crlbCep50M));
                    drawPill(dl, sx, sy, buf, eCol, placed, k, true, false);
                }
            } else if (!labeledBearing) {
                const vsg::dvec3 orig(e.originEcef[0], e.originEcef[1], e.originEcef[2]);
                vsg::dvec3 dir(e.rayDirEcef[0], e.rayDirEcef[1], e.rayDirEcef[2]);
                const double dn = vsg::length(dir);
                if (dn > 1e-6) {
                    dir /= dn;
                    const double len = std::clamp(static_cast<double>(e.rayLengthM), 7000.0, 22000.0);
                    std::vector<vsg::dvec3> cands{orig + dir * (len * 0.48), orig + dir * (len * 0.72), orig + dir * (len * 0.28)};
                    float sx = 0.0f, sy = 0.0f;
                    if (pickAdaptiveAnchor(cands, panelBoxes, k, sx, sy)) {
                        drawPill(dl, sx, sy, warn ? "告警测向线" : (comint ? "通信侦察测向" : "电子侦察测向"),
                                 eCol, placed, k, true, false);
                        labeledBearing = true;
                    }
                }
            }
        }
    }

    // 4. Focused Vehicle Weapon DLZ / WEZ Color-Coded Zone Labels (NEZ + Rmax + LCOS/CCIP)
    if (mask & kOverlayWeapons) {
        for (const auto& w : frame_.weapons) {
            if (static_cast<int>(w.slot) != selectedSlot_) continue;

            if ((w.flags & ipc::kTactWeaponHasDlz) != 0u && w.rMaxM > 100.0f) {
                const vsg::dvec3 orig(states_[w.slot].positionEcef[0], states_[w.slot].positionEcef[1], states_[w.slot].positionEcef[2]);
                vsg::dvec3 east, north, up;
                localEnuVec(orig, east, north, up);
                vsg::dvec3 aimDir = bodyToEcefVec(states_[w.slot], vsg::dvec3(1.0, 0.0, 0.0));
                const vsg::dvec3 tgt(w.targetEcef[0], w.targetEcef[1], w.targetEcef[2]);
                if (vsg::length2(tgt - orig) > 100.0) aimDir = vsg::normalize(tgt - orig);
                vsg::dvec3 horiz = aimDir - up * vsg::dot(aimDir, up);
                if (vsg::length(horiz) < 1e-4) horiz = north;
                horiz = vsg::normalize(horiz);
                const vsg::dvec3 right = vsg::normalize(vsg::cross(horiz, up));

                const double rMin = std::max(200.0, static_cast<double>(w.rMinM));
                const double rNe  = std::max(rMin + 200.0, static_cast<double>(w.rNeM));
                const double rMax = std::min(95000.0, std::max(rNe + 200.0, static_cast<double>(w.rMaxM)));
                const bool inNez = (w.flags & ipc::kTactWeaponInNez) != 0u;
                const double halfAng = 0.24;
                auto fanPt = [&](double dist, double uFrac) {
                    const double a = halfAng * uFrac;
                    return orig + (horiz * std::cos(a) + right * std::sin(a)) * dist;
                };

                // (a) Emerald NEZ Zone Label
                const double rNeMid = rMin + (rNe - rMin) * 0.55;
                std::vector<vsg::dvec3> nezCands{
                    fanPt(rNeMid, 0.0), fanPt(rNeMid, -0.55), fanPt(rNeMid, 0.55),
                    fanPt(rMin + (rNe - rMin) * 0.78, 0.0), fanPt(rMin + (rNe - rMin) * 0.35, 0.0)};
                float sx = 0.0f, sy = 0.0f;
                if (pickAdaptiveAnchor(nezCands, panelBoxes, k, sx, sy)) {
                    char buf[88];
                    std::snprintf(buf, sizeof buf, "%s 不可逃逸区 %.0f km%s",
                                  w.storeName[0] ? w.storeName : "空空导弹",
                                  rNe * 1e-3, inNez ? " [可发射]" : "");
                    drawPill(dl, sx, sy, buf, {42, 224, 128}, placed, k, false, true);
                }

                // (b) Amber Rmax Outer Zone Label
                if (rMax > rNe + 2500.0) {
                    const double rMaxMid = rNe + (rMax - rNe) * 0.45;
                    std::vector<vsg::dvec3> maxCands{
                        fanPt(rMaxMid, 0.0), fanPt(rMaxMid, -0.55), fanPt(rMaxMid, 0.55),
                        fanPt(rNe + (rMax - rNe) * 0.25, 0.0), fanPt(rNe + (rMax - rNe) * 0.70, 0.0)};
                    if (pickAdaptiveAnchor(maxCands, panelBoxes, k, sx, sy)) {
                        char buf[64];
                        std::snprintf(buf, sizeof buf, "最大射程区 %.0f km", rMax * 1e-3);
                        drawPill(dl, sx, sy, buf, {245, 190, 58}, placed, k, false, true);
                    }
                }
            }
            if ((w.flags & ipc::kTactWeaponHasLcos) != 0u) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(w.lcosPipperEcef[0], w.lcosPipperEcef[1], w.lcosPipperEcef[2]), sx, sy)) {
                    const ImU32 col = colour(80, 240, 146, 235);
                    dl->AddCircle(ImVec2(sx, sy), 9.0f * k, colour(10, 16, 24, 200), 24, 2.8f * k);
                    dl->AddCircle(ImVec2(sx, sy), 9.0f * k, col, 24, 1.5f * k);
                    dl->AddCircleFilled(ImVec2(sx, sy), 2.0f * k, col);
                    drawPill(dl, sx, sy, "机炮前置瞄准点", {80, 240, 146}, placed, k, false, false);
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
                    drawPill(dl, sx, sy, "炸弹预测落点 CCIP", {255, 198, 52}, placed, k, false, false);
                }
            }
        }
    }

    // 5. Focused Vehicle Link 16 / C2 / PNT Zone Labels
    if (mask & kOverlayLinks) {
        bool labeledLink16 = false;
        for (const auto& l : frame_.links) {
            const auto kind = static_cast<ipc::TacticalLinkKindV1>(l.kind);
            const bool involvesSelected =
                (static_cast<int>(l.fromSlot) == selectedSlot_) ||
                ((kind == ipc::TacticalLinkKindV1::Link16Dms || kind == ipc::TacticalLinkKindV1::C2Consent ||
                  kind == ipc::TacticalLinkKindV1::Mode5IffReply) &&
                 static_cast<int>(l.toSlot) == selectedSlot_);
            if (!involvesSelected) continue;

            const vsg::dvec3 pFrom(l.fromEcef[0], l.fromEcef[1], l.fromEcef[2]);
            const vsg::dvec3 pTo(l.toEcef[0], l.toEcef[1], l.toEcef[2]);
            const vsg::dvec3 mid = (pFrom + pTo) * 0.5;
            const double span = vsg::length(pTo - pFrom);
            const vsg::dvec3 upMid = localUpVec(mid);
            const double arch = std::min(span * 0.07, 1600.0);

            if (kind == ipc::TacticalLinkKindV1::Link16Dms && !labeledLink16 && span > 150.0) {
                std::vector<vsg::dvec3> cands{
                    mid + upMid * arch,
                    pFrom + (pTo - pFrom) * 0.32 + upMid * (arch * 0.87),
                    pFrom + (pTo - pFrom) * 0.68 + upMid * (arch * 0.87)};
                float sx = 0.0f, sy = 0.0f;
                if (pickAdaptiveAnchor(cands, panelBoxes, k, sx, sy)) {
                    drawPill(dl, sx, sy, "数据链 Link 16", {46, 228, 164}, placed, k, false, true);
                    labeledLink16 = true;
                }
            } else if (kind == ipc::TacticalLinkKindV1::C2Consent) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(mid + upMid * arch, sx, sy))
                    drawPill(dl, sx, sy, "C2 授权链路", {255, 206, 56}, placed, k, false, true);
            } else if (kind == ipc::TacticalLinkKindV1::PntGnssDrift && (l.flags & ipc::kTactLinkJammed) != 0u) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(pFrom, sx, sy)) {
                    char buf[56];
                    std::snprintf(buf, sizeof buf, "导航干扰误差圈 %.0f m", static_cast<double>(l.param));
                    drawPill(dl, sx, sy, buf, {245, 74, 62}, placed, k, false, false);
                }
            }
        }
    }

    // 6. Focused Vehicle SAR / Weather Swath Zone Labels
    if (mask & kOverlaySwaths) {
        for (const auto& s : frame_.swaths) {
            if (static_cast<int>(s.slot) != selectedSlot_) continue;
            const vsg::dvec3 center(s.centerEcef[0], s.centerEcef[1], s.centerEcef[2]);
            const vsg::dvec3 axE(s.axisEastEcef[0], s.axisEastEcef[1], s.axisEastEcef[2]);
            const vsg::dvec3 axN(s.axisNorthEcef[0], s.axisNorthEcef[1], s.axisNorthEcef[2]);
            std::vector<vsg::dvec3> cands{
                center,
                center + axE * (static_cast<double>(s.halfWidthM) * 0.45),
                center - axE * (static_cast<double>(s.halfWidthM) * 0.45),
                center + axN * (static_cast<double>(s.halfLengthM) * 0.45)};
            float sx = 0.0f, sy = 0.0f;
            if (pickAdaptiveAnchor(cands, panelBoxes, k, sx, sy)) {
                char buf[72];
                if (s.kind == static_cast<std::uint8_t>(ipc::TacticalSwathKindV1::WeatherCell)) {
                    std::snprintf(buf, sizeof buf, "气象回波区 %.0f dBZ", static_cast<double>(s.metricDb));
                    drawPill(dl, sx, sy, buf, {245, 190, 56}, placed, k, false, true);
                } else {
                    std::snprintf(buf, sizeof buf, "SAR 成像区 %.1f m · 噪底 %.0f dB",
                                  static_cast<double>(s.resolutionM), static_cast<double>(s.metricDb));
                    drawPill(dl, sx, sy, buf, {56, 208, 236}, placed, k, false, true);
                }
            }
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    lastDrawMs_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

} // namespace fsim::ui
