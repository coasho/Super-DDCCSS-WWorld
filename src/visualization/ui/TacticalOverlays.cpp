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

std::uint64_t makeLabelKey(std::uint32_t slot, std::uint32_t category, std::uint32_t subIdx) noexcept {
    return (static_cast<std::uint64_t>(slot) << 24u) |
           (static_cast<std::uint64_t>(category) << 12u) |
           static_cast<std::uint64_t>(subIdx & 0xFFFu);
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
    if (selectedSlot != lastSelectedSlot_) {
        labelStates_.clear();
        lastSelectedSlot_ = selectedSlot;
    }
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

bool TacticalOverlays::pickStableAnchor(std::uint64_t labelKey,
                                        const std::vector<vsg::dvec3>& candidates,
                                        const std::vector<PlacedBox>& reservedPanels,
                                        float scale,
                                        float& outAnchorX, float& outAnchorY) const noexcept {
    struct CandEval {
        int idx = -1;
        float x = 0.0f, y = 0.0f;
        float score = 1e30f;
        bool safe = false;
    };

    const float w = static_cast<float>(width_);
    const float h = static_cast<float>(height_);
    const float padX = 76.0f * scale;
    const float padY = 44.0f * scale;

    std::vector<CandEval> evals;
    evals.reserve(candidates.size());

    for (std::size_t i = 0; i < candidates.size(); ++i) {
        float sx = 0.0f, sy = 0.0f;
        if (!projectPoint(candidates[i], sx, sy)) continue;
        if (sx < 8.0f || sx > w - 8.0f || sy < 8.0f || sy > h - 8.0f) continue;

        float pen = 0.0f;
        if (sx < padX) pen += (padX - sx) * 2.2f;
        if (sx > w - padX) pen += (sx - (w - padX)) * 2.2f;
        if (sy < padY) pen += (padY - sy) * 2.2f;
        if (sy > h - padY) pen += (sy - (h - padY)) * 2.2f;

        bool insidePanel = false;
        for (const auto& b : reservedPanels) {
            if (sx >= b.x0 - 16.0f * scale && sx <= b.x1 + 16.0f * scale &&
                sy >= b.y0 - 12.0f * scale && sy <= b.y1 + 12.0f * scale) {
                insidePanel = true;
                pen += 2200.0f;
                break;
            }
        }
        // Slight preference for earlier (primary) candidates so anchor selection is naturally deterministic.
        pen += static_cast<float>(i) * 18.0f * scale;
        evals.push_back({static_cast<int>(i), sx, sy, pen, !insidePanel && pen < 120.0f * scale});
    }
    if (evals.empty()) return false;

    auto& st = labelStates_[labelKey];
    const CandEval* prev = nullptr;
    const CandEval* best = &evals.front();
    for (const auto& e : evals) {
        if (e.score < best->score) best = &e;
        if (e.idx == st.candIdx) prev = &e;
    }

    // Hysteresis: stick with the previous 3-D candidate unless the new candidate beats it by a large margin
    // (or the previous candidate entered a HUD panel / left the viewport). Eliminates candidate flip-flopping!
    const CandEval* chosen = best;
    if (prev && prev->safe && best->score + 220.0f * scale > prev->score) {
        chosen = prev;
    }
    st.candIdx = chosen->idx;
    outAnchorX = chosen->x;
    outAnchorY = chosen->y;
    return true;
}

void TacticalOverlays::drawZoneTag(void* drawList, std::uint64_t labelKey,
                                   float anchorX, float anchorY,
                                   float preferOffsetX, float preferOffsetY,
                                   const char* text, ZoneColor zc,
                                   std::vector<PlacedBox>& placed,
                                   float scale, double nowWallS, float dtS,
                                   bool drawLeader) const {
    if (!text || !text[0]) return;
    auto* dl = static_cast<ImDrawList*>(drawList);
    ImFont* font = ImGui::GetFont();
    // Lighter typographic weight (0.86x) than solid Entity badges so 3-D regions remain the visual hero!
    const float fontSize = std::max(11.5f, ImGui::GetFontSize() * 0.86f);
    const ImVec2 sz = font->CalcTextSizeA(fontSize, 2000.0f, 0.0f, text);
    const float padX = 4.0f * scale;
    const float padY = 2.0f * scale;
    const float boxW = sz.x + padX * 2.0f;
    const float boxH = sz.y + padY * 2.0f;

    const bool alignRight = (preferOffsetX < 0.0f);
    float targetX = alignRight ? (anchorX + preferOffsetX - boxW) : (anchorX + preferOffsetX);
    float targetY = anchorY + preferOffsetY - boxH * 0.5f;

    const float minX = 14.0f * scale;
    const float maxX = std::max(minX + 4.0f, static_cast<float>(width_) - boxW - 14.0f * scale);
    const float minY = 14.0f * scale;
    const float maxY = std::max(minY + 4.0f, static_cast<float>(height_) - boxH - 14.0f * scale);
    targetX = std::clamp(targetX, minX, maxX);
    targetY = std::clamp(targetY, minY, maxY);

    // Deterministic non-overlapping placement against panels, entity tags, and higher-priority zone labels.
    const float gap = 6.0f * scale;
    for (int iter = 0; iter < 10; ++iter) {
        bool hit = false;
        for (const auto& b : placed) {
            if (targetX < b.x1 + gap && targetX + boxW + gap > b.x0 &&
                targetY < b.y1 + gap && targetY + boxH + gap > b.y0) {
                if (b.x0 > static_cast<float>(width_) * 0.55f && targetX + boxW > b.x0) {
                    targetX = std::max(minX, b.x0 - boxW - gap);
                } else if (b.y0 > static_cast<float>(height_) * 0.65f) {
                    targetY = std::max(minY, b.y0 - boxH - gap);
                } else if (targetY + boxH + gap + boxH > maxY) {
                    targetY = std::max(minY, b.y0 - boxH - gap);
                } else {
                    targetY = b.y1 + gap;
                }
                hit = true;
                break;
            }
        }
        if (!hit) break;
    }
    targetX = std::clamp(targetX, minX, maxX);
    targetY = std::clamp(targetY, minY, maxY);

    // Critically-damped screen smoothing + deadzone hysteresis:
    // The thin leader line attaches instantaneously to (anchorX, anchorY), while the text card itself
    // stays calm and steady (eliminating high-frequency jitter and frame-to-frame jumping!).
    auto& st = labelStates_[labelKey];
    if (!st.valid || (nowWallS - st.lastWallS) > 0.6) {
        st.textX = targetX;
        st.textY = targetY;
        st.valid = true;
    } else {
        const float dx = targetX - st.textX;
        const float dy = targetY - st.textY;
        const float dist = std::hypot(dx, dy);
        if (dist > 340.0f * scale) {
            // Instant snap on camera mode switch / teleport
            st.textX = targetX;
            st.textY = targetY;
        } else if (dist > 3.0f * scale) {
            // Smooth exponential damping outside deadzone
            const float alpha = 1.0f - std::exp(-std::clamp(dtS, 0.004f, 0.08f) * 7.5f);
            st.textX += dx * alpha;
            st.textY += dy * alpha;
        }
    }
    st.lastWallS = nowWallS;

    // Final safety nudge ONLY if smoothed position actively collides with a reserved box
    float x0 = std::clamp(st.textX, minX, maxX);
    float y0 = std::clamp(st.textY, minY, maxY);
    for (const auto& b : placed) {
        if (x0 < b.x1 + 2.0f && x0 + boxW + 2.0f > b.x0 && y0 < b.y1 + 2.0f && y0 + boxH + 2.0f > b.y0) {
            x0 = targetX;
            y0 = targetY;
            st.textX = x0;
            st.textY = y0;
            break;
        }
    }

    const ImVec2 lo(x0, y0);
    const ImVec2 hi(x0 + boxW, y0 + boxH);
    placed.push_back({lo.x, lo.y, hi.x, hi.y});

    // Quiet, borderless typographic zone annotation (no bulky opaque card obscuring the 3-D zone!):
    // Crisp dark text halo + high-luminance zone-tinted typography + subtle hairline underline + leader line.
    const int txtR = std::clamp((255 * 52 + zc.r * 48) / 100, 165, 255);
    const int txtG = std::clamp((255 * 52 + zc.g * 48) / 100, 165, 255);
    const int txtB = std::clamp((255 * 52 + zc.b * 48) / 100, 165, 255);
    const ImU32 lineCol  = colour(zc.r, zc.g, zc.b, 215);
    const ImU32 underCol = colour(zc.r, zc.g, zc.b, 128);
    const ImU32 haloCol  = colour(5, 9, 16, 224);
    const ImU32 textCol  = colour(txtR, txtG, txtB, 246);

    const float underY = hi.y - 1.0f;
    if (drawLeader) {
        const float elbowX = (anchorX < lo.x) ? lo.x : (anchorX > hi.x ? hi.x : (lo.x + hi.x) * 0.5f);
        const float elbowY = underY;
        const float leadLen = std::hypot(anchorX - elbowX, anchorY - elbowY);
        if (leadLen > 5.0f * scale && leadLen < 420.0f * scale) {
            dl->AddLine(ImVec2(anchorX, anchorY), ImVec2(elbowX, elbowY), colour(6, 10, 18, 170), 2.2f * scale);
            dl->AddLine(ImVec2(anchorX, anchorY), ImVec2(elbowX, elbowY), lineCol, 1.0f * scale);
            dl->AddCircleFilled(ImVec2(anchorX, anchorY), 2.4f * scale, lineCol);
        }
    }

    // Whisper-subtle hairline underline (gives clean anchoring without an opaque dark rectangle!)
    dl->AddLine(ImVec2(lo.x + padX, underY), ImVec2(hi.x - padX, underY), colour(6, 10, 18, 150), 2.0f);
    dl->AddLine(ImVec2(lo.x + padX, underY), ImVec2(hi.x - padX, underY), underCol, 1.0f);

    // 4-way dark outline for effortless legibility against both bright white clouds and dark sea
    const ImVec2 tp(lo.x + padX, lo.y + padY);
    const float o = std::max(1.0f, 1.15f * scale);
    dl->AddText(font, fontSize, ImVec2(tp.x - o, tp.y), haloCol, text);
    dl->AddText(font, fontSize, ImVec2(tp.x + o, tp.y), haloCol, text);
    dl->AddText(font, fontSize, ImVec2(tp.x, tp.y - o), haloCol, text);
    dl->AddText(font, fontSize, ImVec2(tp.x, tp.y + o), haloCol, text);
    dl->AddText(font, fontSize, tp, textCol, text);
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
    const double nowWallS = std::chrono::duration<double>(t0.time_since_epoch()).count();
    const float dtS = lastRecordWallS_ > 0.0 ? static_cast<float>(std::clamp(nowWallS - lastRecordWallS_, 0.004, 0.08)) : (1.0f / 60.0f);
    lastRecordWallS_ = nowWallS;

    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const float k = uiScale();
    const float w = static_cast<float>(width_);
    const float h = static_cast<float>(height_);

    // Reserve exact live workspace dock bounds reported by MonitorGui / MissionGui!
    std::vector<PlacedBox> panelBoxes;
    panelBoxes.reserve(4);
    const float topBarY = controls_ ? controls_->topBarBottomPx.load(std::memory_order_relaxed) : 0.0f;
    const float topBarR = controls_ ? controls_->topBarRightPx.load(std::memory_order_relaxed) : 0.0f;
    const float rightDockX = controls_ ? controls_->rightDockLeftPx.load(std::memory_order_relaxed) : 0.0f;
    const float leftDockR = controls_ ? controls_->leftDockRightPx.load(std::memory_order_relaxed) : 0.0f;
    const float leftDockT = controls_ ? controls_->leftDockTopPx.load(std::memory_order_relaxed) : 0.0f;

    panelBoxes.push_back({0.0f, 0.0f, topBarR > 10.0f ? topBarR + 8.0f * k : w * 0.78f,
                          topBarY > 10.0f ? topBarY + 8.0f * k : 115.0f * k});
    if (leftDockR > 10.0f && leftDockT > 10.0f) {
        panelBoxes.push_back({0.0f, leftDockT - 8.0f * k, leftDockR + 8.0f * k, h});
    }
    if (rightDockX > 10.0f && rightDockX < w - 10.0f) {
        panelBoxes.push_back({rightDockX - 10.0f * k, 0.0f, w, h});
    } else {
        panelBoxes.push_back({std::max(0.0f, w - std::min(410.0f * k, w * 0.30f)), 0.0f, w, h * 0.65f});
    }

    std::vector<PlacedBox> placed = panelBoxes;
    placed.reserve(44);
    // Reserve solid entity badge zones (with generous halo so quiet zone text never touches aircraft callsigns!)
    for (std::size_t i = 0; i < states_.size() && i < alive_.size(); ++i) {
        if (!alive_[i]) continue;
        float vx = 0.0f, vy = 0.0f;
        const vsg::dvec3 p(states_[i].positionEcef[0], states_[i].positionEcef[1], states_[i].positionEcef[2]);
        if (projectPoint(p, vx, vy)) {
            placed.push_back({vx - 14.0f * k, vy - 20.0f * k, vx + 130.0f * k, vy + 20.0f * k});
        }
    }

    auto sideOfSlot = [&](std::uint32_t slot) {
        if (slot < meta_.size()) return world::TacticalVolumes::inferSide(meta_[slot].name);
        return world::TacticalVolumes::Side::Neutral;
    };
    auto sideZoneColor = [](world::TacticalVolumes::Side side) -> ZoneColor {
        if (side == world::TacticalVolumes::Side::Red)  return {248, 86, 86};
        if (side == world::TacticalVolumes::Side::Blue) return {62, 188, 255};
        return {145, 208, 235};
    };

    bool focusedRadarJammed = false;

    // 1. Focused Vehicle Radar Zones (spread cleanly onto opposite flanks to stay out of the center target line!)
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
            const double elVisual = std::clamp(static_cast<double>(r.elevationHalfWidthRad), 0.050, 0.125);
            auto dirAt = [&](double az, double el) {
                return bFwd * (std::cos(el) * std::cos(az)) + bRight * (std::cos(el) * std::sin(az)) + bUp * std::sin(el);
            };

            // (a) Active Radar Zone Label -> anchored on the starboard/right rail so it never crowds center target
            const double activeOuter = hasBurn ? static_cast<double>(r.burnThroughRangeM) : nomRange;
            std::vector<vsg::dvec3> activeCands;
            if (roto) {
                vsg::dvec3 e, n, u;
                localEnuVec(origin, e, n, u);
                for (int i = 0; i < 6; ++i) {
                    const double a = i * (3.141592653589793 / 3.0);
                    activeCands.push_back(origin + (e * std::cos(a) + n * std::sin(a)) * (activeOuter * 0.65));
                }
            } else {
                activeCands.push_back(origin + dirAt(azC + azH * 0.92, elC + elVisual * 0.7) * (activeOuter * 0.68));
                activeCands.push_back(origin + dirAt(azC + azH * 0.88, elC) * std::min(activeOuter * 0.55, 6500.0));
                activeCands.push_back(origin + dirAt(azC - azH * 0.92, elC + elVisual * 0.7) * (activeOuter * 0.68));
                activeCands.push_back(origin + dirAt(azC - azH * 0.88, elC) * std::min(activeOuter * 0.55, 6500.0));
            }
            float ax = 0.0f, ay = 0.0f;
            const std::uint64_t keyActive = makeLabelKey(r.slot, 1u, r.payloadIndex);
            if (pickStableAnchor(keyActive, activeCands, panelBoxes, k, ax, ay)) {
                char buf[72];
                if (hasBurn) {
                    std::snprintf(buf, sizeof buf, "雷达烧穿区 %.1f km", static_cast<double>(r.burnThroughRangeM) * 1e-3);
                } else {
                    const char* prefix = (side == world::TacticalVolumes::Side::Red)  ? "红方雷达区"
                                       : (side == world::TacticalVolumes::Side::Blue) ? "蓝方雷达区"
                                                                                      : "雷达搜索区";
                    std::snprintf(buf, sizeof buf, "%s %.0f km", prefix, rawRange * 1e-3);
                }
                const float sidePush = (ax >= w * 0.5f) ? 26.0f * k : -26.0f * k;
                drawZoneTag(dl, keyActive, ax, ay, sidePush, -14.0f * k, buf, radarCol, placed, k, nowWallS, dtS, true);
            }

            // (b) Burn-Through Gate & Suppressed Outer Blind Zone -> anchored cleanly on the port/left portal rim
            if (hasBurn) {
                const double btR = static_cast<double>(r.burnThroughRangeM);
                const double blindR = btR + (nomRange - btR) * 0.55;
                std::vector<vsg::dvec3> btCands;
                std::vector<vsg::dvec3> blindCands;
                if (!roto) {
                    btCands.push_back(origin + dirAt(azC - azH * 0.92, elC + elVisual * 0.9) * btR);
                    btCands.push_back(origin + dirAt(azC + azH * 0.92, elC + elVisual * 0.9) * btR);
                    btCands.push_back(origin + dirAt(azC - azH * 0.45, elC + elVisual * 0.8) * btR);
                    btCands.push_back(origin + dirAt(azC + azH * 0.45, elC + elVisual * 0.8) * btR);
                    blindCands.push_back(origin + dirAt(azC - azH * 0.72, elC + elVisual) * blindR);
                    blindCands.push_back(origin + dirAt(azC + azH * 0.72, elC + elVisual) * blindR);
                    blindCands.push_back(origin + dirAt(azC - azH * 0.90, elC) * (btR + (nomRange - btR) * 0.25));
                } else {
                    vsg::dvec3 e, n, u;
                    localEnuVec(origin, e, n, u);
                    for (int i = 0; i < 6; ++i) {
                        const double a = i * (3.141592653589793 / 3.0);
                        btCands.push_back(origin + (e * std::cos(a) + n * std::sin(a)) * btR);
                        blindCands.push_back(origin + (e * std::cos(a) + n * std::sin(a)) * blindR);
                    }
                }
                const std::uint64_t keyBt = makeLabelKey(r.slot, 2u, r.payloadIndex);
                if (pickStableAnchor(keyBt, btCands, panelBoxes, k, ax, ay)) {
                    char buf[72];
                    std::snprintf(buf, sizeof buf, "烧穿边界 %.1f km · +%0.f dB",
                                  btR * 1e-3, static_cast<double>(r.jammerRatioDb));
                    const float sidePush = (ax <= w * 0.5f) ? -28.0f * k : 28.0f * k;
                    drawZoneTag(dl, keyBt, ax, ay, sidePush, -18.0f * k, buf, {255, 208, 56}, placed, k, nowWallS, dtS, true);
                }
                const std::uint64_t keyBlind = makeLabelKey(r.slot, 3u, r.payloadIndex);
                if (nomRange - btR > 2500.0 && pickStableAnchor(keyBlind, blindCands, panelBoxes, k, ax, ay)) {
                    const float sidePush = (ax <= w * 0.5f) ? -20.0f * k : 20.0f * k;
                    drawZoneTag(dl, keyBlind, ax, ay, sidePush, -20.0f * k, "干扰压制盲区", {186, 198, 220}, placed, k, nowWallS, dtS, true);
                }
            }

            // (c) Target Track Diamond (clean crisp symbol; text only if STT/Illuminating)
            if ((r.flags & ipc::kTactRadarHasTarget) != 0u) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(r.targetEcef[0], r.targetEcef[1], r.targetEcef[2]), sx, sy)) {
                    const float rad = (stt || illum ? 9.5f : 6.5f) * k;
                    const ImVec2 pts[4] = {{sx, sy - rad}, {sx + rad, sy}, {sx, sy + rad}, {sx - rad, sy}};
                    dl->AddPolyline(pts, 4, colour(8, 14, 22, 210), ImDrawFlags_Closed, 2.8f * k);
                    dl->AddPolyline(pts, 4, colour(radarCol.r, radarCol.g, radarCol.b, 255), ImDrawFlags_Closed, 1.4f * k);
                    if (stt || illum) {
                        char buf[56];
                        std::snprintf(buf, sizeof buf, "%s %.1f km", illum ? "照射引导" : "单目标锁定",
                                      static_cast<double>(r.targetRangeM) * 1e-3);
                        drawZoneTag(dl, makeLabelKey(r.slot, 4u, r.payloadIndex), sx, sy, 22.0f * k, -16.0f * k,
                                    buf, radarCol, placed, k, nowWallS, dtS, false);
                    }
                }
            }
        }
    }

    // 2. EA Jammer Zone Label
    if (mask & kOverlayJammer) {
        const sim::VehicleState& selSt = states_[static_cast<std::size_t>(selectedSlot_)];
        const vsg::dvec3 selPos(selSt.positionEcef[0], selSt.positionEcef[1], selSt.positionEcef[2]);
        std::uint32_t jIdx = 0;
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

            std::vector<vsg::dvec3> cands;
            if ((j.flags & ipc::kTactJamAllRound) != 0u) {
                vsg::dvec3 e, n, u;
                localEnuVec(origin, e, n, u);
                const double rad = rng * 0.35;
                cands = {origin + e * rad, origin - e * rad, origin + n * rad, origin - n * rad};
            } else {
                const double d1 = rng * 0.48, w1 = d1 * std::tan(halfAng) * 0.88;
                const double d2 = rng * 0.68, w2 = d2 * std::tan(halfAng) * 0.88;
                cands = {
                    origin + dir * d1 - sideVec * w1,
                    origin + dir * d1 + sideVec * w1,
                    origin + dir * d2 - sideVec * w2,
                    origin + dir * d2 + sideVec * w2};
            }
            float sx = 0.0f, sy = 0.0f;
            const std::uint64_t keyJ = makeLabelKey(j.slot, 5u, jIdx++);
            if (pickStableAnchor(keyJ, cands, panelBoxes, k, sx, sy)) {
                char buf[72];
                std::snprintf(buf, sizeof buf, "%s %.0f dBW · %.1f GHz", comms ? "通信干扰区" : "电子干扰区",
                              static_cast<double>(j.erpDbw), static_cast<double>(j.frequencyGhz));
                const ZoneColor jCol = comms ? ZoneColor{248, 158, 52} : ZoneColor{206, 114, 255};
                const float sidePush = (sx <= w * 0.5f) ? -22.0f * k : 22.0f * k;
                drawZoneTag(dl, keyJ, sx, sy, sidePush, 0.0f, buf, jCol, placed, k, nowWallS, dtS, true);
            }
        }
    }

    // 3. Focused Vehicle ESM Bearing & CRLB Ellipsoid Zone Labels
    if (mask & kOverlayEsm) {
        bool labeledBearing = false;
        std::uint32_t eIdx = 0;
        for (const auto& e : frame_.esmRays) {
            if (static_cast<int>(e.observerSlot) != selectedSlot_) continue;
            const bool warn = (e.flags & ipc::kTactEsmWarning) != 0u;
            const bool comint = (e.flags & ipc::kTactEsmComint) != 0u;
            const ZoneColor eCol = warn ? ZoneColor{250, 92, 72}
                                 : comint ? ZoneColor{62, 236, 170}
                                          : ZoneColor{252, 212, 72};

            if ((e.flags & ipc::kTactEsmHasGeo) != 0u && e.crlbCep50M > 1.0f) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(e.geoPosEcef[0], e.geoPosEcef[1], e.geoPosEcef[2]), sx, sy)) {
                    char buf[64];
                    if (e.crlbCep50M >= 1000.0f) std::snprintf(buf, sizeof buf, "定位误差椭球 %.2f km", static_cast<double>(e.crlbCep50M) * 1e-3);
                    else std::snprintf(buf, sizeof buf, "定位误差椭球 %.0f m", static_cast<double>(e.crlbCep50M));
                    drawZoneTag(dl, makeLabelKey(e.observerSlot, 6u, eIdx++), sx, sy, 22.0f * k, 12.0f * k,
                                buf, eCol, placed, k, nowWallS, dtS, true);
                }
            } else if (!labeledBearing) {
                const vsg::dvec3 orig(e.originEcef[0], e.originEcef[1], e.originEcef[2]);
                vsg::dvec3 dir(e.rayDirEcef[0], e.rayDirEcef[1], e.rayDirEcef[2]);
                const double dn = vsg::length(dir);
                if (dn > 1e-6) {
                    dir /= dn;
                    const double len = std::clamp(static_cast<double>(e.rayLengthM), 7000.0, 22000.0);
                    std::vector<vsg::dvec3> cands{orig + dir * (len * 0.56), orig + dir * (len * 0.34)};
                    float sx = 0.0f, sy = 0.0f;
                    const std::uint64_t keyE = makeLabelKey(e.observerSlot, 7u, 0u);
                    if (pickStableAnchor(keyE, cands, panelBoxes, k, sx, sy)) {
                        drawZoneTag(dl, keyE, sx, sy, 22.0f * k, 16.0f * k,
                                    warn ? "告警测向线" : (comint ? "通信侦察测向" : "电子侦察测向"),
                                    eCol, placed, k, nowWallS, dtS, true);
                        labeledBearing = true;
                    }
                }
            }
        }
    }

    // 4. Focused Vehicle Weapon DLZ Upright Range-Gate Labels (anchored cleanly to left/right gate posts!)
    if (mask & kOverlayWeapons) {
        for (const auto& wCue : frame_.weapons) {
            if (static_cast<int>(wCue.slot) != selectedSlot_) continue;

            if ((wCue.flags & ipc::kTactWeaponHasDlz) != 0u && wCue.rMaxM > 100.0f) {
                const vsg::dvec3 orig(states_[wCue.slot].positionEcef[0], states_[wCue.slot].positionEcef[1], states_[wCue.slot].positionEcef[2]);
                vsg::dvec3 east, north, up;
                localEnuVec(orig, east, north, up);
                vsg::dvec3 aimDir = bodyToEcefVec(states_[wCue.slot], vsg::dvec3(1.0, 0.0, 0.0));
                const vsg::dvec3 tgt(wCue.targetEcef[0], wCue.targetEcef[1], wCue.targetEcef[2]);
                if (vsg::length2(tgt - orig) > 100.0) aimDir = vsg::normalize(tgt - orig);
                vsg::dvec3 horiz = aimDir - up * vsg::dot(aimDir, up);
                if (vsg::length(horiz) < 1e-4) horiz = north;
                horiz = vsg::normalize(horiz);
                const vsg::dvec3 right = vsg::normalize(vsg::cross(horiz, up));
                const vsg::dvec3 deckOrigin = orig - up * 35.0;

                const double rMin = std::max(200.0, static_cast<double>(wCue.rMinM));
                const double rNe  = std::max(rMin + 200.0, static_cast<double>(wCue.rNeM));
                const double rMax = std::min(95000.0, std::max(rNe + 200.0, static_cast<double>(wCue.rMaxM)));
                const bool inNez = (wCue.flags & ipc::kTactWeaponInNez) != 0u;
                const double halfAng = 0.22;

                auto gatePostTop = [&](double dist, double sideSign, double wScale) {
                    const double a = halfAng * wScale * sideSign;
                    const double gateH = std::clamp(dist * 0.026, 65.0, 520.0);
                    return deckOrigin + (horiz * std::cos(a) + right * std::sin(a)) * dist + up * (gateH * 0.7);
                };

                // NEZ -> anchored on right/starboard gate post (with mid-range fallback for steep close-up views!)
                float sx = 0.0f, sy = 0.0f;
                const std::uint64_t keyNez = makeLabelKey(wCue.slot, 8u, 0u);
                std::vector<vsg::dvec3> nezCands{
                    gatePostTop(rNe, 1.0, 0.94),
                    gatePostTop(rNe, -1.0, 0.94),
                    gatePostTop(std::min(rNe, 9500.0), 1.0, 0.94),
                    gatePostTop(std::min(rNe, 5200.0), 1.0, 0.94)};
                if (pickStableAnchor(keyNez, nezCands, panelBoxes, k, sx, sy)) {
                    char buf[80];
                    std::snprintf(buf, sizeof buf, "NEZ 不可逃逸 %.0f km%s", rNe * 1e-3, inNez ? " [可发射]" : "");
                    const float sidePush = (sx >= w * 0.5f) ? 26.0f * k : -26.0f * k;
                    drawZoneTag(dl, keyNez, sx, sy, sidePush, 10.0f * k, buf, {58, 232, 138}, placed, k, nowWallS, dtS, true);
                }

                // Rmax -> anchored on left/port gate post (with mid-range fallback when outer arc is off-frustum)
                if (rMax > rNe + 2500.0) {
                    const std::uint64_t keyMax = makeLabelKey(wCue.slot, 8u, 1u);
                    std::vector<vsg::dvec3> maxCands{
                        gatePostTop(rMax, -1.0, 1.0),
                        gatePostTop(rMax, 1.0, 1.0),
                        gatePostTop(std::min(rMax, std::max(rNe * 1.15, 14000.0)), -1.0, 1.0)};
                    if (pickStableAnchor(keyMax, maxCands, panelBoxes, k, sx, sy)) {
                        char buf[64];
                        std::snprintf(buf, sizeof buf, "Rmax 最大射程 %.0f km", rMax * 1e-3);
                        const float sidePush = (sx <= w * 0.5f) ? -26.0f * k : 26.0f * k;
                        drawZoneTag(dl, keyMax, sx, sy, sidePush, 12.0f * k, buf, {248, 196, 68}, placed, k, nowWallS, dtS, true);
                    }
                }
            }
            if ((wCue.flags & ipc::kTactWeaponHasLcos) != 0u) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(wCue.lcosPipperEcef[0], wCue.lcosPipperEcef[1], wCue.lcosPipperEcef[2]), sx, sy)) {
                    const ImU32 col = colour(80, 240, 146, 225);
                    dl->AddCircle(ImVec2(sx, sy), 6.5f * k, colour(8, 14, 22, 190), 20, 2.4f * k);
                    dl->AddCircle(ImVec2(sx, sy), 6.5f * k, col, 20, 1.2f * k);
                    dl->AddCircleFilled(ImVec2(sx, sy), 1.8f * k, col);
                }
            }
            if ((wCue.flags & ipc::kTactWeaponHasCcip) != 0u) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(vsg::dvec3(wCue.ccipImpactEcef[0], wCue.ccipImpactEcef[1], wCue.ccipImpactEcef[2]), sx, sy)) {
                    const ImU32 col = colour(255, 198, 52, 240);
                    dl->AddCircle(ImVec2(sx, sy), 8.0f * k, colour(8, 14, 22, 200), 20, 2.4f * k);
                    dl->AddCircle(ImVec2(sx, sy), 8.0f * k, col, 20, 1.4f * k);
                    drawZoneTag(dl, makeLabelKey(wCue.slot, 9u, 0u), sx, sy, 20.0f * k, 0.0f,
                                "炸弹落点 CCIP", {255, 204, 64}, placed, k, nowWallS, dtS, false);
                }
            }
        }
    }

    // 5. Focused Vehicle Link 16 / C2 / PNT
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
                std::vector<vsg::dvec3> cands{mid + upMid * arch, pFrom + (pTo - pFrom) * 0.35 + upMid * (arch * 0.9)};
                float sx = 0.0f, sy = 0.0f;
                const std::uint64_t keyL = makeLabelKey(static_cast<std::uint32_t>(selectedSlot_), 10u, 0u);
                if (pickStableAnchor(keyL, cands, panelBoxes, k, sx, sy)) {
                    drawZoneTag(dl, keyL, sx, sy, 16.0f * k, -14.0f * k, "数据链 Link 16", {56, 232, 170}, placed, k, nowWallS, dtS, true);
                    labeledLink16 = true;
                }
            } else if (kind == ipc::TacticalLinkKindV1::C2Consent) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(mid + upMid * arch, sx, sy))
                    drawZoneTag(dl, makeLabelKey(l.fromSlot, 10u, 1u), sx, sy, 16.0f * k, -14.0f * k,
                                "C2 授权链路", {255, 208, 60}, placed, k, nowWallS, dtS, true);
            } else if (kind == ipc::TacticalLinkKindV1::PntGnssDrift && (l.flags & ipc::kTactLinkJammed) != 0u) {
                float sx = 0.0f, sy = 0.0f;
                if (projectPoint(pFrom, sx, sy)) {
                    char buf[56];
                    std::snprintf(buf, sizeof buf, "导航干扰误差 %.0f m", static_cast<double>(l.param));
                    drawZoneTag(dl, makeLabelKey(l.fromSlot, 10u, 2u), sx, sy, 18.0f * k, 18.0f * k,
                                buf, {248, 86, 72}, placed, k, nowWallS, dtS, true);
                }
            }
        }
    }

    // 6. Focused Vehicle SAR / Weather Swaths
    if (mask & kOverlaySwaths) {
        std::uint32_t sIdx = 0;
        for (const auto& s : frame_.swaths) {
            if (static_cast<int>(s.slot) != selectedSlot_) continue;
            const vsg::dvec3 center(s.centerEcef[0], s.centerEcef[1], s.centerEcef[2]);
            const vsg::dvec3 axE(s.axisEastEcef[0], s.axisEastEcef[1], s.axisEastEcef[2]);
            std::vector<vsg::dvec3> cands{center + axE * (static_cast<double>(s.halfWidthM) * 0.5), center};
            float sx = 0.0f, sy = 0.0f;
            const std::uint64_t keyS = makeLabelKey(s.slot, 11u, sIdx++);
            if (pickStableAnchor(keyS, cands, panelBoxes, k, sx, sy)) {
                char buf[72];
                if (s.kind == static_cast<std::uint8_t>(ipc::TacticalSwathKindV1::WeatherCell)) {
                    std::snprintf(buf, sizeof buf, "气象回波 %.0f dBZ", static_cast<double>(s.metricDb));
                    drawZoneTag(dl, keyS, sx, sy, 20.0f * k, 0.0f, buf, {248, 196, 62}, placed, k, nowWallS, dtS, true);
                } else {
                    std::snprintf(buf, sizeof buf, "SAR 成像区 %.1f m", static_cast<double>(s.resolutionM));
                    drawZoneTag(dl, keyS, sx, sy, 20.0f * k, 0.0f, buf, {64, 214, 240}, placed, k, nowWallS, dtS, true);
                }
            }
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    lastDrawMs_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

} // namespace fsim::ui
