#include "visualization/ui/MissionGui.h"
#include "visualization/ui/Colour.h"
#include "visualization/ui/MandatoryPanel.h"

#include "foundation/core/Units.h"
#include "foundation/transport/WorldMirror.h"

#include <vsgImGui/imgui.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace fsim::ui {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

const char* stateName(std::uint8_t s) {
    static const char* names[] = {"关闭", "启动中", "待机", "工作", "自检", "故障"};
    return s < 6 ? names[s] : "?";
}

ImVec4 stateColour(std::uint8_t s) {
    switch (s) {
    case 1: return tint(1.0f, 0.85f, 0.30f);
    case 2: return tint(0.45f, 0.80f, 1.00f);
    case 3: return tint(0.35f, 0.94f, 0.54f);
    case 4: return tint(1.0f, 0.62f, 0.22f);
    case 5: return tint(1.0f, 0.34f, 0.30f);
    default: return tint(0.58f, 0.64f, 0.72f);
    }
}

const char* modeName(std::uint8_t m) {
    static const char* names[] = {"RWS", "VS", "TWS", "SURV", "STT"};
    return m < 5 ? names[m] : "?";
}

const char* modeLong(std::uint8_t m) {
    static const char* names[] = {"边搜索边测距", "速度搜索", "边扫描边跟踪", "全向警戒", "单目标锁定"};
    return m < 5 ? names[m] : "?";
}

const char* emitterModeName(std::uint8_t m) {
    static const char* names[] = {"搜索雷达", "跟踪雷达", "照射雷达", "干扰机", "数据链", "导引头"};
    return m < 6 ? names[m] : "辐射源";
}

void bodyAzEl(const sim::VehicleState& own, const double targetEcef[3], double& azRad, double& elRad, double& distM) noexcept {
    const double dx = targetEcef[0] - own.positionEcef[0];
    const double dy = targetEcef[1] - own.positionEcef[1];
    const double dz = targetEcef[2] - own.positionEcef[2];
    distM = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double* R = own.rotationBodyToEcef;
    const double fwd = R[0] * dx + R[3] * dy + R[6] * dz;
    const double right = R[1] * dx + R[4] * dy + R[7] * dz;
    const double down = R[2] * dx + R[5] * dy + R[8] * dz;
    azRad = std::atan2(right, fwd);
    elRad = std::atan2(-down, std::hypot(fwd, right));
}

double bodyRayAzimuthDeg(const sim::VehicleState& own, const float rayDirEcef[3]) noexcept {
    const double* R = own.rotationBodyToEcef;
    const double dx = static_cast<double>(rayDirEcef[0]);
    const double dy = static_cast<double>(rayDirEcef[1]);
    const double dz = static_cast<double>(rayDirEcef[2]);
    const double fwd = R[0] * dx + R[3] * dy + R[6] * dz;
    const double right = R[1] * dx + R[4] * dy + R[7] * dz;
    return std::atan2(right, fwd) / kDeg;
}

/// Authentic original green-phosphor Range-Doppler heatmap ramp (pure black floor -> green -> yellow -> hot white).
ImU32 phosphorGreenHeat(int q) {
    const float t = std::clamp(static_cast<float>(q) / 160.0f, 0.0f, 1.0f);
    const float r = std::clamp(2.0f * t - 0.8f, 0.0f, 1.0f);
    const float g = std::clamp(1.6f * t, 0.0f, 1.0f);
    const float b = std::clamp(3.0f * t - 2.0f, 0.0f, 1.0f);
    return colour(static_cast<int>(255.0f * r), static_cast<int>(255.0f * g), static_cast<int>(255.0f * b), 255);
}

} // namespace

bool MissionGui::supportsAirborneRadar(const std::string& aircraftType) noexcept {
    if (aircraftType.empty()) return false;
    std::string lo;
    lo.reserve(aircraftType.size());
    for (char ch : aircraftType) {
        if (ch == '-' || ch == '_' || ch == ' ') continue;
        lo.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    }
    if (lo.rfind("jsbsim:", 0) == 0) lo = lo.substr(7);

    for (const char* deny : {"aim", "pl1", "pl2", "r73", "r77", "r27", "mk8", "gbu", "agm", "jd", "sdb",
                             "missile", "bomb", "rocket", "torpedo", "chaff", "flare",
                             "uh60", "uh1", "ah64", "ch47", "mi8", "mi24", "ka52", "ec135", "heli",
                             "c172", "cessna", "pa28", "a10", "mq9", "quad", "uav", "drone"}) {
        if (lo.find(deny) != std::string::npos) return false;
    }
    for (const char* allow : {"f14", "f15", "f16", "f18", "fa18", "ea18", "f22", "f35", "f4", "f5",
                              "su27", "su30", "su33", "su34", "su35", "su57", "mig29", "mig31", "mig35",
                              "j10", "j11", "j15", "j16", "j20", "j35", "jf17",
                              "mirage", "rafale", "typhoon", "ef2000", "gripen", "jas39", "tornado",
                              "e2", "e3", "e7", "kj500", "kj2000", "awacs", "a50",
                              "b1", "b2", "b52", "h6", "tu95", "tu160", "p8", "c130"}) {
        if (lo.find(allow) != std::string::npos) return true;
    }
    return false;
}

MissionGui::MissionGui(std::shared_ptr<ViewerControls> controls)
    : controls_(std::move(controls)), record_(std::make_unique<ipc::MissionRecord>()),
      fusion_(std::make_unique<ipc::FusionRecordV1>()) {}

void MissionGui::cacheFocusedTactical(const ipc::TacticalOverlayFrameV1* tactical, int slot) {
    focusedRadars_.clear();
    ownJammers_.clear();
    focusedEsm_.clear();
    focusedWeapons_.clear();
    focusedLinks_.clear();
    focusedProp_.clear();
    if (!tactical || slot < 0) return;

    for (const auto& r : tactical->radars) {
        if (static_cast<int>(r.slot) == slot) focusedRadars_.push_back(r);
    }
    // STRICTLY own-platform EA transmitter only (never peek enemy jammer truth!)
    for (const auto& j : tactical->jammers) {
        if (static_cast<int>(j.slot) == slot) ownJammers_.push_back(j);
    }
    // STRICTLY own-platform RWR/ESM receiver intercepts only
    for (const auto& e : tactical->esmRays) {
        if (static_cast<int>(e.observerSlot) == slot) focusedEsm_.push_back(e);
    }
    for (const auto& w : tactical->weapons) {
        if (static_cast<int>(w.slot) == slot) focusedWeapons_.push_back(w);
    }
    for (const auto& l : tactical->links) {
        const auto kind = static_cast<ipc::TacticalLinkKindV1>(l.kind);
        const bool match = (static_cast<int>(l.fromSlot) == slot) ||
                           ((kind == ipc::TacticalLinkKindV1::Link16Dms || kind == ipc::TacticalLinkKindV1::C2Consent ||
                             kind == ipc::TacticalLinkKindV1::Mode5IffReply) &&
                            static_cast<int>(l.toSlot) == slot);
        if (match) focusedLinks_.push_back(l);
    }
    for (const auto& p : tactical->propulsion) {
        if (static_cast<int>(p.slot) == slot) focusedProp_.push_back(p);
    }
}

void MissionGui::update(const ipc::WorldMirror* mirror,
                        int slot,
                        const std::string& name,
                        double simTime,
                        const ipc::TacticalOverlayFrameV1* tactical,
                        const sim::VehicleState* ownState,
                        const std::string& type) {
    replayMode_ = false;
    fusionValid_ = false;
    mirrored_ = mirror && mirror->valid();
    simTime_ = simTime;
    name_ = name;
    type_ = type;
    hasOwnState_ = (ownState != nullptr);
    if (ownState) ownState_ = *ownState;

    if (!mirrored_ || slot < 0) {
        valid_ = false;
        slot_ = -1;
        supportsRadar_ = false;
        cacheFocusedTactical(nullptr, -1);
        return;
    }
    if (slot != slot_) {
        valid_ = false;
        slot_ = slot;
    }
    if (mirror->mission(static_cast<std::uint32_t>(slot), *record_, valid_ ? record_->revision : ~0ull)) valid_ = true;
    fusionValid_ = mirror->fusion(static_cast<std::uint32_t>(slot), *fusion_);
    cacheFocusedTactical(tactical, slot);
    supportsRadar_ = (valid_ && record_->radarCount > 0) || !focusedRadars_.empty();
}

void MissionGui::updateReplay(const ipc::TacticalOverlayFrameV1* tactical,
                              const sim::VehicleState* ownState,
                              int slot,
                              const std::string& name,
                              const std::string& type,
                              double simTime) {
    mirrored_ = false;
    valid_ = false;
    fusionValid_ = false;
    slot_ = slot;
    name_ = name;
    type_ = type;
    simTime_ = simTime;
    hasOwnState_ = (ownState != nullptr);
    if (ownState) ownState_ = *ownState;

    replayMode_ = (slot >= 0);
    if (!replayMode_) {
        supportsRadar_ = false;
        cacheFocusedTactical(nullptr, -1);
        return;
    }
    cacheFocusedTactical(tactical, slot);
    supportsRadar_ = !focusedRadars_.empty() || supportsAirborneRadar(type_);
}

void MissionGui::drawDismissButton(std::uint32_t panelBit, std::uint32_t activePanels) const {
    if (!controls_) return;
    const float btnW = ImGui::CalcTextSize("隐藏").x + ImGui::GetStyle().FramePadding.x * 2.0f;
    const float availX = ImGui::GetContentRegionAvail().x;
    if (panelBit == kPanelRadar || panelBit == kPanelSignal) {
        if (availX > btnW + 8.0f) ImGui::SameLine(ImGui::GetCursorPosX() + availX - btnW);
    } else {
        if (availX > btnW + 8.0f) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + availX - btnW);
    }
    ImGui::PushStyleColor(ImGuiCol_Button, tint(0.14f, 0.19f, 0.27f, 0.70f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, tint(0.42f, 0.16f, 0.18f, 0.90f));
    if (ImGui::SmallButton("隐藏")) {
        controls_->customPanelMask.store(activePanels & ~panelBit, std::memory_order_relaxed);
        controls_->workspacePreset.store(static_cast<int>(WorkspacePreset::Custom), std::memory_order_relaxed);
    }
    ImGui::PopStyleColor(2);
}

void MissionGui::record(vsg::CommandBuffer&) const {
    if (!mirrored_ && !replayMode_) {
        if (controls_) controls_->rightDockLeftPx.store(0.0f, std::memory_order_relaxed);
        return;
    }

    const float k = uiScale();
    const float pScale = controls_ ? std::clamp(controls_->panelScale.load(std::memory_order_relaxed), 0.75f, 1.35f) : 1.0f;
    const float dockBaseW = controls_ ? std::clamp(controls_->rightDockWidthPx.load(std::memory_order_relaxed), 280.0f, 520.0f) : 370.0f;
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float margin = 10.0f * k;
    const auto preset = static_cast<WorkspacePreset>(
        controls_ ? std::clamp(controls_->workspacePreset.load(std::memory_order_relaxed), 0, kWorkspacePresetCount - 1)
                  : static_cast<int>(WorkspacePreset::AirRadar));
    const std::uint32_t customMask = controls_ ? controls_->customPanelMask.load(std::memory_order_relaxed) : 0u;
    const std::uint32_t activePanels = panelsForWorkspace(preset, customMask);

    const bool hasNativeRd = mirrored_ && valid_ && record_->rangeDoppler.radar < record_->radarCount &&
                             record_->radars[record_->rangeDoppler.radar].signalLevel &&
                             record_->rangeDoppler.rows > 0 && record_->rangeDoppler.columns > 0;

    struct DockSlot {
        std::uint32_t bit = 0u;
        float weight = 1.0f;
    };
    std::vector<DockSlot> docked;
    docked.reserve(6);

    // 1. Airborne Radar: normal toggleable plugin (capability-gated to radar aircraft; NEVER forced!)
    if ((activePanels & kPanelRadar) != 0u && supportsRadar_) {
        docked.push_back({kPanelRadar, 1.20f});
    }
    // 2. Authentic Native Signal-Level Range-Doppler: strictly real MissionRangeDoppler ONLY!
    //    (Hidden automatically in preset modes when no signal-level matrix is present; never fabricated!)
    if ((activePanels & kPanelSignal) != 0u && supportsRadar_ &&
        (hasNativeRd || preset == WorkspacePreset::Custom)) {
        docked.push_back({kPanelSignal, hasNativeRd ? 1.15f : 0.45f});
    }
    // 3. Own-Platform EA Transmitter & Own RWR/ESM Receiver Intercepts (zero enemy truth peeking!)
    if ((activePanels & kPanelEw) != 0u && (!ownJammers_.empty() || !focusedEsm_.empty() || !focusedRadars_.empty())) {
        docked.push_back({kPanelEw, 0.90f});
    }
    // 4. Own Fire-Control DLZ / LCOS / CCIP & Mounted Stores
    if ((activePanels & kPanelWeapon) != 0u && (!focusedWeapons_.empty() || (valid_ && record_->storeCount > 0) || !supportsRadar_)) {
        docked.push_back({kPanelWeapon, 0.90f});
    }
    // 5. Own 6-DOF Aerodynamics & Propulsion (up to 8 engines / rotor)
    if ((activePanels & kPanelAero) != 0u && hasOwnState_) {
        docked.push_back({kPanelAero, 0.95f});
    }
    // 6. Own Link 16 / C2 / IFF / PNT & Native Multi-Sensor Fusion
    if ((activePanels & kPanelComms) != 0u && (!focusedLinks_.empty() || fusionValid_ || mirrored_)) {
        docked.push_back({kPanelComms, 0.90f});
    }

    if (docked.empty()) {
        if (controls_) controls_->rightDockLeftPx.store(0.0f, std::memory_order_relaxed);
        return;
    }

    const float colW = std::min(dockBaseW * k * (0.60f + 0.40f * pScale), display.x * 0.42f);
    const float gap = 8.0f * k;
    const float topBarY = controls_ ? controls_->topBarBottomPx.load(std::memory_order_relaxed) : 0.0f;
    const float startY = std::max({display.y * 0.095f, topBarY + gap, margin});
    const float availH = std::max(180.0f * k, display.y - margin - startY);

    const std::size_t col1Count = std::min<std::size_t>(docked.size(), 3u);
    const std::size_t col2Count = docked.size() > 3u ? (docked.size() - 3u) : 0u;
    const float col1X = display.x - margin - colW;
    const float col2X = col1X - gap - colW;
    if (controls_) {
        controls_->rightDockLeftPx.store(col2Count > 0u ? col2X : col1X, std::memory_order_relaxed);
    }

    auto dispatchColumn = [&](std::size_t beginIdx, std::size_t count, float colX) {
        if (count == 0u) return;
        float totalW = 0.0f;
        for (std::size_t i = 0; i < count; ++i) totalW += docked[beginIdx + i].weight;
        const float usableH = std::max(120.0f * k, availH - gap * static_cast<float>(count - 1u));
        const bool isScope = (count == 1u) && (docked[beginIdx].bit == kPanelRadar || docked[beginIdx].bit == kPanelSignal);
        const float maxSingleH = (count == 1u) ? std::min(usableH, (isScope ? 460.0f : 235.0f) * k * pScale) : usableH;
        float curY = startY;
        for (std::size_t i = 0; i < count; ++i) {
            const float share = (count == 1u) ? maxSingleH : (usableH * (docked[beginIdx + i].weight / totalW));
            const DockRect r{colX, curY, colW, share, pScale};
            curY += share + gap;
            switch (docked[beginIdx + i].bit) {
            case kPanelRadar:  drawRadarPanel(r, activePanels); break;
            case kPanelSignal: drawRangeDopplerPanel(r, activePanels); break;
            case kPanelEw:     drawEwPanel(r, activePanels); break;
            case kPanelWeapon: drawWeaponPanel(r, activePanels); break;
            case kPanelAero:   drawAeroPanel(r, activePanels); break;
            case kPanelComms:  drawCommsPanel(r, activePanels); break;
            default: break;
            }
        }
    };

    dispatchColumn(0u, col1Count, col1X);
    if (col2Count > 0u) dispatchColumn(col1Count, col2Count, col2X);
}

// ============================================================================
// Plugin 1: Airborne Radar B-Scope / PPI (Optional, Strictly Own-Radar Data)
// ============================================================================
void MissionGui::drawRadarPanel(const DockRect& rect, std::uint32_t activePanels) const {
    ImGui::SetNextWindowPos(ImVec2(rect.x, rect.y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(rect.w, rect.h), ImGuiCond_Always);
    char title[128];
    std::snprintf(title, sizeof title, "机载雷达 - %s###radar0", name_.empty() ? "?" : name_.c_str());
    if (!beginMandatoryPanel(title, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::End();
        return;
    }
    ImGui::SetWindowFontScale(rect.fontScale);

    if (mirrored_ && valid_ && record_->radarCount > 0) {
        drawLiveRadarBody(record_->radars[0]);
    } else if (!focusedRadars_.empty()) {
        drawReplayRadarBody(focusedRadars_[0]);
    } else {
        drawStandbyRadarBody();
    }
    drawDismissButton(kPanelRadar, activePanels);
    ImGui::End();
}

void MissionGui::drawLiveRadarBody(const ipc::MissionRadar& r) const {
    const float k = uiScale();
    const ipc::MissionPayload* p = r.payload < record_->payloadCount ? &record_->payloads[r.payload] : nullptr;
    const bool operating = p && p->state == 3;
    const ipc::TacticalRadarVolumeWireV1* matchedTact = focusedRadars_.empty() ? nullptr : &focusedRadars_[0];
    const bool jammed = matchedTact && (matchedTact->flags & ipc::kTactRadarJammed) != 0u;

    if (r.searching) {
        ImGui::TextColored(tint(0.35f, 0.94f, 0.54f), "%s", modeName(r.mode));
        ImGui::SameLine();
        ImGui::Text("%s · %.0f km", modeLong(r.mode), static_cast<double>(r.rangeScaleM) * 1e-3);
        if (r.illuminating) { ImGui::SameLine(); ImGui::TextColored(warningText(), "照射"); }
        ImGui::TextDisabled("%.2f GHz · %s%s", r.frequencyHz * 1e-9,
                            r.rotodome ? "旋转雷达罩" : r.electronic ? "AESA 相控阵" : "机械扫描",
                            r.signalLevel ? " · 信号级" : "");
    } else {
        ImGui::TextColored(stateColour(p ? p->state : 0), "%s", operating ? "待机（未搜索）" : p ? stateName(p->state) : "?");
    }
    if (jammed) {
        ImGui::TextColored(cautionText(), "本机接收底噪抬升 +%.0f dB（烧穿 %.1f km）",
                           static_cast<double>(matchedTact->jammerRatioDb),
                           static_cast<double>(matchedTact->burnThroughRangeM) * 1e-3);
    }

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float footerH = 1.6f * ImGui::GetTextLineHeightWithSpacing();
    const float h = std::max(85.0f * k, avail.y - footerH);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 size(avail.x, h);
    ImGui::InvisibleButton("radar_scope", size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);
    dl->AddRectFilled(p0, p1, colour(6, 15, 20, 250), 4.0f * k);
    dl->AddRect(p0, p1, colour(58, 96, 108, 130), 4.0f * k, 0, 1.0f);

    const double scale = r.rangeScaleM > 0.0f ? static_cast<double>(r.rangeScaleM)
                                              : (r.instrumentedRangeM > 0.0f ? static_cast<double>(r.instrumentedRangeM) : 50000.0);
    const ImU32 grid = colour(30, 62, 62, 185);
    const ImU32 labelCol = colour(128, 174, 174, 240);
    const ImU32 beamCol = r.illuminating ? colour(255, 194, 52, 245) : colour(102, 240, 186, 225);
    const ImU32 trackCol = colour(112, 224, 255, 255);
    const ImU32 hitCol = colour(104, 242, 136, 255);
    char text[48];

    if (!r.rotodome) {
        const double limit = r.azimuthLimitRad > 0.0f ? static_cast<double>(r.azimuthLimitRad) : 60.0 * kDeg;
        auto xOf = [&](double az) { return p0.x + static_cast<float>((az + limit) / (2.0 * limit)) * size.x; };
        auto yOf = [&](double rng) { return p1.y - static_cast<float>(std::clamp(rng / scale, 0.0, 1.0)) * size.y; };

        if (r.searching && r.azimuthHalfWidthRad > 0.0f) {
            const float x0 = std::clamp(xOf(r.azimuthCentreRad - r.azimuthHalfWidthRad), p0.x, p1.x);
            const float x1 = std::clamp(xOf(r.azimuthCentreRad + r.azimuthHalfWidthRad), p0.x, p1.x);
            dl->AddRectFilled(ImVec2(x0, p0.y), ImVec2(x1, p1.y), colour(16, 48, 52, 115));
            if (jammed && matchedTact->burnThroughRangeM > 100.0f && matchedTact->burnThroughRangeM < scale) {
                const float yBt = yOf(matchedTact->burnThroughRangeM);
                dl->AddRectFilled(ImVec2(x0, p0.y), ImVec2(x1, yBt), colour(145, 36, 115, 58));
                dl->AddLine(ImVec2(p0.x, yBt), ImVec2(p1.x, yBt), colour(255, 206, 52, 245), 1.8f * k);
            }
        }
        for (int q = 1; q < 4; ++q) {
            const float y = yOf(scale * static_cast<double>(q) / 4.0);
            dl->AddLine(ImVec2(p0.x, y), ImVec2(p1.x, y), grid);
            std::snprintf(text, sizeof text, "%.0f km", scale * static_cast<double>(q) / 4000.0);
            dl->AddText(ImVec2(p0.x + 4.0f * k, y - ImGui::GetTextLineHeight()), labelCol, text);
        }
        for (int deg : {-60, -30, 0, 30, 60}) {
            const double rad = deg * kDeg;
            if (std::abs(rad) >= limit) continue;
            const float x = xOf(rad);
            dl->AddLine(ImVec2(x, p0.y), ImVec2(x, p1.y), grid);
        }
        if (r.searching) {
            const float bx = std::clamp(xOf(r.beamAzimuthRad), p0.x, p1.x);
            dl->AddLine(ImVec2(bx, p0.y), ImVec2(bx, p1.y), beamCol, 2.0f * k);
        }
        for (std::uint32_t i = 0; i < r.recentCount && i < ipc::kMissionDetections; ++i) {
            const auto& d = r.recent[i];
            if (d.rangeM > scale) continue;
            const double age = std::max(0.0, simTime_ - d.timeS);
            const float fade = static_cast<float>(std::clamp(1.0 - age / std::max(6.0, static_cast<double>(r.framePeriodS) * 2.0), 0.15, 1.0));
            const float x = std::clamp(xOf(d.azimuthRad), p0.x, p1.x);
            const float y = yOf(d.rangeM);
            const float rad = (d.snrDb > 20.0f ? 3.8f : 2.6f) * k;
            dl->AddCircleFilled(ImVec2(x, y), rad, colour(104, 242, 136, static_cast<int>(fade * 235.0f)));
        }
        for (std::uint32_t i = 0; i < r.trackCount && i < ipc::kMissionTracks; ++i) {
            const auto& tr = r.tracks[i];
            if (tr.rangeM > scale) continue;
            const float x = std::clamp(xOf(tr.azimuthRad), p0.x, p1.x);
            const float y = yOf(tr.rangeM);
            const float hbox = 5.0f * k;
            dl->AddRect(ImVec2(x - hbox, y - hbox), ImVec2(x + hbox, y + hbox), trackCol, 0.0f, 0, 1.7f * k);
            if ((tr.flags & ipc::kMissionTrackDesignated) != 0u) {
                const float d = 1.8f * hbox;
                const ImVec2 dia[4] = {{x, y - d}, {x + d, y}, {x, y + d}, {x - d, y}};
                dl->AddPolyline(dia, 4, colour(255, 212, 74, 255), ImDrawFlags_Closed, 1.6f * k);
            }
            std::snprintf(text, sizeof text, "%u", static_cast<unsigned>(tr.id));
            dl->AddText(ImVec2(x + hbox + 3.0f * k, y - ImGui::GetTextLineHeight() * 0.5f), trackCol, text);
        }
    } else {
        const ImVec2 c(p0.x + size.x * 0.5f, p0.y + size.y * 0.5f);
        const float radius = std::min(size.x, size.y) * 0.46f;
        auto at = [&](double az, double rng) {
            const float f = static_cast<float>(std::clamp(rng / scale, 0.0, 1.0)) * radius;
            return ImVec2(c.x + f * static_cast<float>(std::sin(az)), c.y - f * static_cast<float>(std::cos(az)));
        };
        for (int q = 1; q <= 4; ++q) dl->AddCircle(c, radius * static_cast<float>(q) / 4.0f, grid, 56);
        for (int deg = 0; deg < 360; deg += 30) dl->AddLine(c, at(deg * kDeg, scale), grid);
        if (r.searching) dl->AddLine(c, at(r.beamAzimuthRad, scale), beamCol, 2.0f * k);
        for (std::uint32_t i = 0; i < r.recentCount && i < ipc::kMissionDetections; ++i) {
            const auto& d = r.recent[i];
            if (d.rangeM <= scale) dl->AddCircleFilled(at(d.azimuthRad, d.rangeM), 2.8f * k, hitCol);
        }
        for (std::uint32_t i = 0; i < r.trackCount && i < ipc::kMissionTracks; ++i) {
            const auto& tr = r.tracks[i];
            if (tr.rangeM > scale) continue;
            const ImVec2 q = at(tr.azimuthRad, tr.rangeM);
            dl->AddRect(ImVec2(q.x - 4.5f * k, q.y - 4.5f * k), ImVec2(q.x + 4.5f * k, q.y + 4.5f * k), trackCol, 0.0f, 0, 1.6f * k);
        }
    }

    ImGui::TextDisabled("航迹 %u（暂定 %u）· 探测 %llu",
                        static_cast<unsigned>(r.trackCount), static_cast<unsigned>(r.tentativeTracks),
                        static_cast<unsigned long long>(r.detections));
}

void MissionGui::drawReplayRadarBody(const ipc::TacticalRadarVolumeWireV1& r) const {
    const float k = uiScale();
    const bool searching = (r.flags & ipc::kTactRadarSearching) != 0u;
    const bool roto = (r.flags & ipc::kTactRadarRotodome) != 0u;
    const bool elec = (r.flags & ipc::kTactRadarElectronic) != 0u;
    const bool stt = (r.flags & ipc::kTactRadarSttLock) != 0u;
    const bool illum = (r.flags & ipc::kTactRadarIlluminating) != 0u;
    const bool jammed = (r.flags & ipc::kTactRadarJammed) != 0u;
    const bool hasTgt = (r.flags & ipc::kTactRadarHasTarget) != 0u;

    if (searching) {
        ImGui::TextColored(tint(0.35f, 0.94f, 0.54f), "%s", modeName(r.mode));
        ImGui::SameLine();
        ImGui::Text("%s · 量程 %.0f km", modeLong(r.mode), static_cast<double>(r.rangeScaleM) * 1e-3);
        if (stt || illum) { ImGui::SameLine(); ImGui::TextColored(warningText(), illum ? "照射" : "锁定"); }
        ImGui::TextDisabled("%s · 方位 %+.0f°±%.0f°",
                            roto ? "旋转雷达罩" : elec ? "AESA 相控阵" : "机械扫描",
                            static_cast<double>(r.azimuthCentreRad) / kDeg,
                            static_cast<double>(r.azimuthHalfWidthRad) / kDeg);
    } else {
        ImGui::TextColored(stateColour(2), "待机（未搜索）");
    }
    if (jammed && r.burnThroughRangeM > 100.0f) {
        ImGui::TextColored(cautionText(), "本机接收底噪抬升 +%.0f dB（烧穿 %.1f km）",
                           static_cast<double>(r.jammerRatioDb),
                           static_cast<double>(r.burnThroughRangeM) * 1e-3);
    }

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float footerH = 1.6f * ImGui::GetTextLineHeightWithSpacing();
    const float h = std::max(85.0f * k, avail.y - footerH);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 size(avail.x, h);
    ImGui::InvisibleButton("replay_radar_scope", size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);
    dl->AddRectFilled(p0, p1, colour(6, 15, 20, 250), 4.0f * k);
    dl->AddRect(p0, p1, colour(58, 96, 108, 130), 4.0f * k, 0, 1.0f);

    const double scale = std::max(5000.0, static_cast<double>(r.rangeScaleM > 100.0f ? r.rangeScaleM : r.detectionRangeM));
    const ImU32 grid = colour(30, 62, 62, 185);
    const ImU32 labelCol = colour(128, 174, 174, 240);
    char text[48];

    double tgtAz = 0.0, tgtEl = 0.0, tgtRange = static_cast<double>(r.targetRangeM);
    bool tgtValid = false;
    if (hasTgt && hasOwnState_) {
        double distM = 0.0;
        bodyAzEl(ownState_, r.targetEcef, tgtAz, tgtEl, distM);
        if (tgtRange < 1.0) tgtRange = distM;
        tgtValid = std::isfinite(tgtAz) && tgtRange > 1.0;
    }

    if (!roto) {
        const double limit = std::clamp(std::max(static_cast<double>(r.azimuthHalfWidthRad) * 1.4, 60.0 * kDeg), 30.0 * kDeg, 90.0 * kDeg);
        auto xOf = [&](double az) { return p0.x + static_cast<float>((az + limit) / (2.0 * limit)) * size.x; };
        auto yOf = [&](double rng) { return p1.y - static_cast<float>(std::clamp(rng / scale, 0.0, 1.0)) * size.y; };

        if (searching && r.azimuthHalfWidthRad > 0.01f) {
            const float x0 = std::clamp(xOf(static_cast<double>(r.azimuthCentreRad - r.azimuthHalfWidthRad)), p0.x, p1.x);
            const float x1 = std::clamp(xOf(static_cast<double>(r.azimuthCentreRad + r.azimuthHalfWidthRad)), p0.x, p1.x);
            dl->AddRectFilled(ImVec2(x0, p0.y), ImVec2(x1, p1.y), colour(16, 48, 52, 118));
            if (jammed && r.burnThroughRangeM > 100.0f && r.burnThroughRangeM < scale) {
                const float yBt = yOf(static_cast<double>(r.burnThroughRangeM));
                dl->AddRectFilled(ImVec2(x0, p0.y), ImVec2(x1, yBt), colour(145, 36, 115, 62));
                dl->AddLine(ImVec2(p0.x, yBt), ImVec2(p1.x, yBt), colour(255, 206, 52, 245), 1.8f * k);
                std::snprintf(text, sizeof text, "烧穿 %.1f km", static_cast<double>(r.burnThroughRangeM) * 1e-3);
                const ImVec2 tsz = ImGui::CalcTextSize(text);
                dl->AddText(ImVec2(p1.x - tsz.x - 5.0f * k, std::max(p0.y + 3.0f, yBt - tsz.y - 2.0f)),
                            colour(255, 218, 92, 255), text);
            }
        }
        for (int q = 1; q < 4; ++q) {
            const float y = yOf(scale * static_cast<double>(q) / 4.0);
            dl->AddLine(ImVec2(p0.x, y), ImVec2(p1.x, y), grid);
            std::snprintf(text, sizeof text, "%.0f km", scale * static_cast<double>(q) / 4000.0);
            dl->AddText(ImVec2(p0.x + 4.0f * k, y - ImGui::GetTextLineHeight()), labelCol, text);
        }
        for (int deg : {-60, -30, 0, 30, 60}) {
            const double rad = deg * kDeg;
            if (std::abs(rad) >= limit) continue;
            const float x = xOf(rad);
            dl->AddLine(ImVec2(x, p0.y), ImVec2(x, p1.y), grid);
        }
        if (searching) {
            const float bx = std::clamp(xOf(static_cast<double>(r.beamAzimuthRad)), p0.x, p1.x);
            dl->AddLine(ImVec2(bx, p0.y), ImVec2(bx, p1.y),
                        illum ? colour(255, 194, 52, 255) : colour(102, 240, 186, 225), 2.0f * k);
        }
        if (tgtValid && std::abs(tgtAz) <= limit && tgtRange <= scale) {
            const float tx = std::clamp(xOf(tgtAz), p0.x + 5.0f, p1.x - 5.0f);
            const float ty = yOf(tgtRange);
            const float hb = 5.0f * k;
            const ImU32 tCol = colour(112, 224, 255, 255);
            dl->AddRect(ImVec2(tx - hb, ty - hb), ImVec2(tx + hb, ty + hb), tCol, 0.0f, 0, 1.8f * k);
            if (stt || illum) {
                const float d = 1.9f * hb;
                const ImVec2 dia[4] = {{tx, ty - d}, {tx + d, ty}, {tx, ty + d}, {tx - d, ty}};
                dl->AddPolyline(dia, 4, colour(255, 212, 74, 255), ImDrawFlags_Closed, 1.7f * k);
            }
            std::snprintf(text, sizeof text, "%.1f km", tgtRange * 1e-3);
            dl->AddText(ImVec2(tx + hb + 4.0f * k, ty - ImGui::GetTextLineHeight() * 0.5f), tCol, text);
        }
    } else {
        const ImVec2 c(p0.x + size.x * 0.5f, p0.y + size.y * 0.5f);
        const float radius = std::min(size.x, size.y) * 0.46f;
        auto at = [&](double az, double rng) {
            const float f = static_cast<float>(std::clamp(rng / scale, 0.0, 1.0)) * radius;
            return ImVec2(c.x + f * static_cast<float>(std::sin(az)), c.y - f * static_cast<float>(std::cos(az)));
        };
        for (int q = 1; q <= 4; ++q) dl->AddCircle(c, radius * static_cast<float>(q) / 4.0f, grid, 56);
        for (int deg = 0; deg < 360; deg += 30) dl->AddLine(c, at(deg * kDeg, scale), grid);
        if (jammed && r.burnThroughRangeM > 100.0f && r.burnThroughRangeM < scale) {
            dl->AddCircle(c, static_cast<float>(r.burnThroughRangeM / scale) * radius, colour(255, 206, 52, 255), 56, 2.0f * k);
        }
        if (searching) dl->AddLine(c, at(static_cast<double>(r.beamAzimuthRad), scale), colour(102, 240, 186, 220), 2.0f * k);
        if (tgtValid && tgtRange <= scale) {
            const ImVec2 q = at(tgtAz, tgtRange);
            dl->AddRect(ImVec2(q.x - 5.0f * k, q.y - 5.0f * k), ImVec2(q.x + 5.0f * k, q.y + 5.0f * k),
                        colour(112, 224, 255, 255), 0.0f, 0, 1.7f * k);
        }
    }

    ImGui::TextDisabled("波束 %+.1f°/%+.1f°",
                        static_cast<double>(r.beamAzimuthRad) / kDeg,
                        static_cast<double>(r.beamElevationRad) / kDeg);
}

void MissionGui::drawStandbyRadarBody() const {
    const float k = uiScale();
    ImGui::TextColored(stateColour(2), "待机 / 雷达静默");
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float h = std::max(70.0f * k, avail.y - 1.5f * ImGui::GetTextLineHeightWithSpacing());
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 size(avail.x, h);
    ImGui::InvisibleButton("standby_scope", size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);
    dl->AddRectFilled(p0, p1, colour(6, 15, 20, 245), 4.0f * k);
    dl->AddRect(p0, p1, colour(58, 96, 108, 120), 4.0f * k);
    const char* msg = "雷达静默 / 待机";
    const ImVec2 msz = ImGui::CalcTextSize(msg);
    dl->AddText(ImVec2(p0.x + (size.x - msz.x) * 0.5f, p0.y + (size.y - msz.y) * 0.5f), colour(124, 154, 168, 220), msg);
}

// ============================================================================
// Plugin 2: Authentic Native Signal-Level Green-Phosphor Range-Doppler ONLY
//           (Zero fabricated data; original MissionGui 1944c190 phosphor ramp)
// ============================================================================
void MissionGui::drawRangeDopplerPanel(const DockRect& rect, std::uint32_t activePanels) const {
    ImGui::SetNextWindowPos(ImVec2(rect.x, rect.y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(rect.w, rect.h), ImGuiCond_Always);
    const bool hasNativeRd = mirrored_ && valid_ && record_->rangeDoppler.radar < record_->radarCount &&
                             record_->radars[record_->rangeDoppler.radar].signalLevel &&
                             record_->rangeDoppler.rows > 0 && record_->rangeDoppler.columns > 0;
    const ipc::MissionPayload* p = nullptr;
    if (hasNativeRd) {
        const auto& rad = record_->radars[record_->rangeDoppler.radar];
        if (rad.payload < record_->payloadCount) p = &record_->payloads[rad.payload];
    }
    char title[128];
    std::snprintf(title, sizeof title, "原生距离-多普勒 %s###rd_panel", p ? p->designation : "");
    if (!beginMandatoryPanel(title, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::End();
        return;
    }
    ImGui::SetWindowFontScale(rect.fontScale);

    if (hasNativeRd) {
        drawNativeRangeDoppler(record_->rangeDoppler);
    } else {
        ImGui::TextColored(cautionText(), "当前无原生信号级 R-D 矩阵数据");
        ImGui::TextWrapped("为保证严格真实不虚造，本面板仅渲染信号级雷达处理器实时输出的 128×64 距离-多普勒矩阵。"
                           "实时信号级镜像模式下将自动呈现纯黑底绿磷 R-D 画面；回放录制链路将在 VIS-M15 补齐。");
    }
    drawDismissButton(kPanelSignal, activePanels);
    ImGui::End();
}

void MissionGui::drawNativeRangeDoppler(const ipc::MissionRangeDoppler& rd) const {
    const float k = uiScale();
    const int rows = rd.rows, cols = rd.columns;
    ImGui::TextDisabled("PRF %.1f kHz · 模糊距离 %.1f km × %.0f m/s",
                        static_cast<double>(rd.prfHz) * 1e-3,
                        static_cast<double>(rd.ambiguousRangeM) * 1e-3,
                        static_cast<double>(rd.ambiguousRateMs));

    const float axisW = ImGui::CalcTextSize("00.0").x + 4.0f * k;
    const float axisH = ImGui::GetTextLineHeightWithSpacing();
    const float footerH = 1.6f * ImGui::GetTextLineHeightWithSpacing();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    avail.y -= footerH;
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 size(std::max(60.0f * k, avail.x - axisW), std::max(60.0f * k, avail.y - axisH));
    ImGui::InvisibleButton("rd_native", ImVec2(size.x + axisW, size.y + axisH));
    const bool hovered = ImGui::IsItemHovered();
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p0(origin.x + axisW, origin.y), p1(p0.x + size.x, p0.y + size.y);

    // Pure black CRT phosphor canvas (exact original MissionGui appearance)
    dl->AddRectFilled(p0, p1, colour(0, 0, 0, 255));
    const float cw = size.x / static_cast<float>(cols);
    const float ch = size.y / static_cast<float>(rows);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const int q = rd.db[static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) + static_cast<std::size_t>(c)];
            if (q < 12) continue;
            const float x = p0.x + static_cast<float>(c) * cw;
            const float y = p1.y - static_cast<float>(r + 1) * ch;
            dl->AddRectFilled(ImVec2(x, y), ImVec2(x + cw, y + ch), phosphorGreenHeat(q));
        }
    }

    const int filters = cols * rd.columnBin;
    const double va = rd.ambiguousRateMs;
    auto xAt = [&](double closing) {
        double f = closing / va;
        f -= std::floor(f + 0.5);
        return p0.x + static_cast<float>((f + 0.5) * static_cast<double>(filters) / static_cast<double>(rd.columnBin)) * cw;
    };
    auto yAt = [&](double rangeM) { return p1.y - static_cast<float>((rangeM - rd.firstRangeM) / rd.rowM) * ch; };
    const ImU32 grid = colour(60, 70, 80, 255);
    const ImU32 labelCol = colour(150, 160, 170, 255);
    const ImU32 notch = colour(90, 140, 255, 220);

    if (std::isfinite(rd.mainBeamRateMs) && rd.notchMs > 0.0f) {
        const double centre = -static_cast<double>(rd.mainBeamRateMs);
        for (double edge : {centre - rd.notchMs, centre + rd.notchMs}) {
            const float x = xAt(edge);
            dl->AddLine(ImVec2(x, p0.y), ImVec2(x, p1.y), notch, 1.0f * k);
        }
    }
    if (std::isfinite(rd.altitudeM) && rd.notchMs > 0.0f && rd.ambiguousRangeM > 0.0f) {
        const double folded = std::fmod(static_cast<double>(rd.altitudeM), static_cast<double>(rd.ambiguousRangeM));
        const float y = yAt(folded);
        dl->AddRect(ImVec2(xAt(-rd.notchMs), y - 2.5f * static_cast<float>(rd.rowBin) * ch),
                    ImVec2(xAt(rd.notchMs), y + 2.5f * static_cast<float>(rd.rowBin) * ch),
                    notch, 0.0f, 0, 1.0f * k);
    }
    for (std::uint32_t i = 0; i < rd.peakCount && i < ipc::kRangeDopplerPeaks; ++i) {
        const ImVec2 c(p0.x + (static_cast<float>(rd.peakColumn[i]) + 0.5f) * cw,
                       p1.y - (static_cast<float>(rd.peakRow[i]) + 0.5f) * ch);
        dl->AddCircle(c, std::max(4.0f * k, 0.8f * std::max(cw, ch)), colour(255, 80, 200, 255), 16, 1.5f * k);
    }

    char text[32];
    const double top = rd.firstRangeM + static_cast<double>(rows) * static_cast<double>(rd.rowM);
    const double step = top > 20e3 ? 5e3 : top > 8e3 ? 2e3 : 1e3;
    for (double rr = step; rr < top; rr += step) {
        const float y = yAt(rr);
        if (y < p0.y || y > p1.y) continue;
        dl->AddLine(ImVec2(p0.x - 3.0f * k, y), ImVec2(p0.x, y), grid);
        std::snprintf(text, sizeof text, "%.0f", rr * 1e-3);
        dl->AddText(ImVec2(origin.x, y - ImGui::GetTextLineHeight() * 0.5f), labelCol, text);
    }
    const double vStep = va > 300.0 ? 100.0 : va > 120.0 ? 50.0 : 25.0;
    const double halfVa = 0.5 * va;
    for (double v = -std::floor(halfVa / vStep) * vStep; v <= halfVa; v += vStep) {
        const float x = p0.x + static_cast<float>((v / va + 0.5)) * size.x;
        dl->AddLine(ImVec2(x, p1.y), ImVec2(x, p1.y + 3.0f * k), grid);
        std::snprintf(text, sizeof text, "%+.0f", v);
        dl->AddText(ImVec2(x - ImGui::CalcTextSize(text).x * 0.5f, p1.y + 3.0f * k), labelCol, text);
    }
    dl->AddRect(p0, p1, grid);

    if (hovered && mouse.x >= p0.x && mouse.x < p1.x && mouse.y >= p0.y && mouse.y < p1.y) {
        const int c = std::clamp(static_cast<int>((mouse.x - p0.x) / cw), 0, cols - 1);
        const int r = std::clamp(static_cast<int>((p1.y - mouse.y) / ch), 0, rows - 1);
        const int q = rd.db[static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) + static_cast<std::size_t>(c)];
        const double rngKm = (rd.firstRangeM + (static_cast<double>(r) + 0.5) * static_cast<double>(rd.rowM)) * 1e-3;
        const double vel = ((static_cast<double>(c) + 0.5) / static_cast<double>(cols) - 0.5) * va;
        ImGui::TextDisabled("%.1f km · %+.0f m/s · +%.1f dB", rngKm, vel, static_cast<double>(q) * 0.25);
    } else {
        ImGui::TextDisabled("CFAR 峰值 %u · 蓝线=主瓣/高度杂波凹口", static_cast<unsigned>(rd.peakCount));
    }
}

// ============================================================================
// Plugin 3: Strictly Own-Platform EA Transmitter & Own RWR/ESM Receiver ONLY
//           (NEVER peeks enemy jammer ERP, frequency, or true position!)
// ============================================================================
void MissionGui::drawEwPanel(const DockRect& rect, std::uint32_t activePanels) const {
    ImGui::SetNextWindowPos(ImVec2(rect.x, rect.y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(rect.w, rect.h), ImGuiCond_Always);
    if (!beginMandatoryPanel("本机电子战与侦察###ew_panel", ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::End();
        return;
    }
    ImGui::SetWindowFontScale(rect.fontScale);

    // 1. Own platform's OWN active EA transmitter (if focused on an EA aircraft like EA-18G)
    if (!ownJammers_.empty()) {
        for (const auto& j : ownJammers_) {
            ImGui::TextColored(tint(0.84f, 0.56f, 1.0f), "本机干扰发射 · %.1f GHz · %.0f dBW · 带宽 %.0f MHz",
                               static_cast<double>(j.frequencyGhz), static_cast<double>(j.erpDbw),
                               static_cast<double>(j.bandwidthMhz));
        }
    }
    // 2. Own radar receiver front-end noise elevation (strictly own receiver measurement)
    if (!focusedRadars_.empty() && (focusedRadars_[0].flags & ipc::kTactRadarJammed) != 0u) {
        ImGui::TextColored(cautionText(), "雷达接收机实测噪声抬升: +%.1f dB",
                           static_cast<double>(focusedRadars_[0].jammerRatioDb));
    }

    // 3. Strictly own RWR / ESM receiver intercepts (bearing-only or own triangulated CEP50 — never truth range!)
    ImGui::Separator();
    ImGui::Text("本机 RWR / ESM 截获辐射源: %zu 条", focusedEsm_.size());
    if (focusedEsm_.empty()) {
        ImGui::TextDisabled("当前未截获外部辐射源");
    } else if (ImGui::BeginTable("own_esm", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("类型");
        ImGui::TableSetupColumn("相对方位");
        ImGui::TableSetupColumn("接收电平");
        ImGui::TableSetupColumn("定位状态");
        ImGui::TableHeadersRow();
        for (std::size_t i = 0; i < std::min<std::size_t>(focusedEsm_.size(), 6u); ++i) {
            const auto& e = focusedEsm_[i];
            const bool warn = (e.flags & ipc::kTactEsmWarning) != 0u;
            const bool hasGeo = (e.flags & ipc::kTactEsmHasGeo) != 0u && e.crlbCep50M > 1.0f;
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            if (warn) ImGui::TextColored(warningText(), "%s", emitterModeName(e.emitterMode));
            else ImGui::TextUnformatted(emitterModeName(e.emitterMode));
            ImGui::TableSetColumnIndex(1);
            if (hasOwnState_) ImGui::Text("%+.0f°", bodyRayAzimuthDeg(ownState_, e.rayDirEcef));
            else ImGui::TextUnformatted("--");
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.0f dBm", static_cast<double>(e.receivedDbm));
            ImGui::TableSetColumnIndex(3);
            if (hasGeo) ImGui::Text("CEP %.0fm", static_cast<double>(e.crlbCep50M));
            else ImGui::TextDisabled("仅测向");
        }
        ImGui::EndTable();
    }
    ImGui::TextDisabled("完整 360° RWR 威胁方位环将在 VIS-M15 实现");
    drawDismissButton(kPanelEw, activePanels);
    ImGui::End();
}

// ============================================================================
// Plugin 4: Own Fire-Control 5-D Weapon DLZ / LCOS / CCIP & Mounted Stores
// ============================================================================
void MissionGui::drawWeaponPanel(const DockRect& rect, std::uint32_t activePanels) const {
    const float k = uiScale() * rect.fontScale;
    ImGui::SetNextWindowPos(ImVec2(rect.x, rect.y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(rect.w, rect.h), ImGuiCond_Always);
    if (!beginMandatoryPanel("武器包线与挂载###weapon_panel", ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::End();
        return;
    }
    ImGui::SetWindowFontScale(rect.fontScale);

    if (!focusedWeapons_.empty()) {
        const auto& w = focusedWeapons_[0];
        if ((w.flags & ipc::kTactWeaponHasDlz) != 0u && w.rMaxM > 100.0f) {
            const bool inNez = (w.flags & ipc::kTactWeaponInNez) != 0u;
            ImGui::Text("%s", w.storeName[0] ? w.storeName : "空空导弹");
            ImGui::SameLine();
            ImGui::TextColored(inNez ? tint(0.32f, 0.94f, 0.56f) : cautionText(),
                               inNez ? "[不可逃逸区 · 可发射]" : "[射程包线解算中]");

            const float barW = ImGui::GetContentRegionAvail().x;
            const float barH = 20.0f * k;
            const ImVec2 p0 = ImGui::GetCursorScreenPos();
            ImGui::InvisibleButton("dlz_bar", ImVec2(barW, barH));
            ImDrawList* dl = ImGui::GetWindowDrawList();
            const double span = std::max({static_cast<double>(w.rAeroM), static_cast<double>(w.rMaxM), 1000.0});
            auto xOf = [&](double m) { return p0.x + static_cast<float>(std::clamp(m / span, 0.0, 1.0)) * barW; };
            dl->AddRectFilled(p0, ImVec2(p0.x + barW, p0.y + barH), colour(16, 24, 36, 240), 3.0f * k);
            dl->AddRectFilled(p0, ImVec2(xOf(w.rMinM), p0.y + barH), colour(224, 58, 62, 195), 3.0f * k);
            dl->AddRectFilled(ImVec2(xOf(w.rMinM), p0.y), ImVec2(xOf(w.rNeM), p0.y + barH), colour(36, 208, 118, 210));
            dl->AddRectFilled(ImVec2(xOf(w.rNeM), p0.y), ImVec2(xOf(w.rMaxM), p0.y + barH), colour(232, 176, 46, 170));
            if (!focusedRadars_.empty() && (focusedRadars_[0].flags & ipc::kTactRadarHasTarget) != 0u) {
                const float tx = xOf(focusedRadars_[0].targetRangeM);
                dl->AddLine(ImVec2(tx, p0.y - 2.0f), ImVec2(tx, p0.y + barH + 2.0f), colour(255, 255, 255, 255), 2.0f * k);
            }
            dl->AddRect(p0, ImVec2(p0.x + barW, p0.y + barH), colour(82, 112, 146, 150), 3.0f * k);

            ImGui::TextDisabled("Rmin %.1f · NEZ %.1f · Rmax %.1f · Raero %.1f km",
                                static_cast<double>(w.rMinM) * 1e-3, static_cast<double>(w.rNeM) * 1e-3,
                                static_cast<double>(w.rMaxM) * 1e-3, static_cast<double>(w.rAeroM) * 1e-3);
        }
        if ((w.flags & ipc::kTactWeaponHasLcos) != 0u) {
            ImGui::Text("机炮 LCOS: 初速 %.0f m/s · 阻力系数 %.4f",
                        static_cast<double>(w.gunMuzzleSpeedMs), static_cast<double>(w.gunDragK));
        }
        if ((w.flags & ipc::kTactWeaponHasCcip) != 0u) {
            ImGui::Text("炸弹 CCIP 落点预测: 已激活");
        }
    } else if (!supportsRadar_ && hasOwnState_) {
        ImGui::TextColored(cautionText(), "在飞弹药 / 非火控载具: %s", name_.c_str());
        ImGui::Text("速度 %.0f m/s (Ma %.2f) · 过载 %.1f G", ownState_.airspeedTrueMs, ownState_.mach, ownState_.loadFactor);
    } else {
        ImGui::TextDisabled("当前无激活武器火控解算");
    }

    if (mirrored_ && valid_ && record_->storeCount > 0) {
        ImGui::Separator();
        ImGui::TextDisabled("本机外挂 (%u 挂点):", static_cast<unsigned>(record_->storeCount));
        for (std::uint32_t i = 0; i < record_->storeCount && i < ipc::kMissionStores; ++i) {
            const auto& s = record_->stores[i];
            const bool gone = (s.status == 4 || s.status == 5 || s.status == 7);
            if (i > 0) ImGui::SameLine(0.0f, 10.0f * k);
            if (ImGui::GetContentRegionAvail().x < 92.0f * k) ImGui::NewLine();
            if (gone) ImGui::TextDisabled("%u %s", static_cast<unsigned>(s.station), s.mnemonic);
            else ImGui::TextColored(s.status == 2 ? tint(0.35f, 0.94f, 0.54f) : tint(0.86f, 0.90f, 0.96f),
                                    "%u %s", static_cast<unsigned>(s.station), s.mnemonic);
        }
    }
    drawDismissButton(kPanelWeapon, activePanels);
    ImGui::End();
}

// ============================================================================
// Plugin 5: Own 6-DOF Aerodynamics, Flight Controls & up to 8-Engine / Rotor
// ============================================================================
void MissionGui::drawAeroPanel(const DockRect& rect, std::uint32_t activePanels) const {
    const float k = uiScale() * rect.fontScale;
    ImGui::SetNextWindowPos(ImVec2(rect.x, rect.y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(rect.w, rect.h), ImGuiCond_Always);
    if (!beginMandatoryPanel("气动舵面与推进###aero_panel", ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::End();
        return;
    }
    ImGui::SetWindowFontScale(rect.fontScale);

    if (!hasOwnState_) {
        ImGui::TextDisabled("暂无状态数据");
        drawDismissButton(kPanelAero, activePanels);
        ImGui::End();
        return;
    }
    const auto& s = ownState_;
    ImGui::Text("Ma %.2f · 表速 %.0f m/s · 过载 %.2f G", s.mach, s.airspeedCalibratedMs, s.loadFactor);
    ImGui::TextDisabled("迎角 %+.1f° · 侧滑 %+.1f° · 角速率 %+.0f/%+.0f/%+.0f °/s",
                        units::radiansToDegrees(s.alphaRad), units::radiansToDegrees(s.betaRad),
                        units::radiansToDegrees(s.angularRateBodyRadS[0]),
                        units::radiansToDegrees(s.angularRateBodyRadS[1]),
                        units::radiansToDegrees(s.angularRateBodyRadS[2]));

    auto barRow = [&](const char* label, float frac01, const char* valTxt, ImU32 col) {
        const float lblW = 68.0f * k;
        const float valW = 86.0f * k;
        const float barW = std::max(36.0f * k, ImGui::GetContentRegionAvail().x - lblW - valW - 10.0f * k);
        const float rowH = ImGui::GetTextLineHeight();
        const ImVec2 p = ImGui::GetCursorScreenPos();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddText(p, colour(175, 194, 212, 245), label);
        const ImVec2 b0(p.x + lblW, p.y + rowH * 0.22f);
        const ImVec2 b1(b0.x + barW, p.y + rowH * 0.82f);
        dl->AddRectFilled(b0, b1, colour(22, 32, 46, 235), 2.5f * k);
        dl->AddRectFilled(b0, ImVec2(b0.x + barW * std::clamp(frac01, 0.0f, 1.0f), b1.y), col, 2.5f * k);
        dl->AddText(ImVec2(b1.x + 8.0f * k, p.y), colour(232, 240, 248, 250), valTxt);
        ImGui::Dummy(ImVec2(ImGui::GetContentRegionAvail().x, rowH + 2.0f * k));
    };

    double totalThrustKn = 0.0;
    double maxN2 = 0.0;
    double maxAb = 0.0;
    int engCount = 0;
    if (!focusedProp_.empty() && focusedProp_[0].engineCount > 0) {
        const auto& pr = focusedProp_[0];
        engCount = std::min<int>(pr.engineCount, ipc::kTacticalMaxEnginesV1);
        for (int i = 0; i < engCount; ++i) {
            totalThrustKn += static_cast<double>(pr.thrustN[i]) * 1e-3;
            maxN2 = std::max(maxN2, static_cast<double>(pr.engineN2[i]));
            maxAb = std::max(maxAb, static_cast<double>(pr.afterburner[i]));
        }
    } else {
        engCount = std::clamp(s.engineCount, 0, sim::VehicleState::kMaxEngines);
        for (int i = 0; i < engCount; ++i) {
            totalThrustKn += s.thrustN[i] * 1e-3;
            maxN2 = std::max(maxN2, s.engineN2[i]);
            maxAb = std::max(maxAb, s.afterburner[i]);
        }
    }

    char vbuf[48];
    if (engCount > 0) {
        std::snprintf(vbuf, sizeof vbuf, "%.1f kN (%d发)", totalThrustKn, engCount);
        barRow("总推力", static_cast<float>(totalThrustKn / std::max(80.0, 110.0 * engCount)), vbuf, colour(245, 168, 48, 240));
        std::snprintf(vbuf, sizeof vbuf, "%.0f%% / AB %.0f%%", maxN2, maxAb * 100.0);
        barRow("核心转速", static_cast<float>(maxN2 / 105.0), vbuf, colour(62, 188, 255, 240));
    }
    if (!focusedProp_.empty() && focusedProp_[0].rotorRpm > 1.0f) {
        std::snprintf(vbuf, sizeof vbuf, "%.0f RPM", static_cast<double>(focusedProp_[0].rotorRpm));
        barRow("旋翼/桨速", static_cast<float>(focusedProp_[0].rotorRpm / 400.0f), vbuf, colour(68, 224, 152, 240));
    }
    std::snprintf(vbuf, sizeof vbuf, "%+.1f° / %+.1f°", units::radiansToDegrees(s.aileronRad), units::radiansToDegrees(s.elevatorRad));
    barRow("副翼/升降", static_cast<float>(0.5 + units::radiansToDegrees(s.elevatorRad) / 40.0), vbuf, colour(168, 134, 252, 240));
    if (std::isfinite(s.fuelKg) && s.fuelKg > 0.0) {
        std::snprintf(vbuf, sizeof vbuf, "%.0f kg", s.fuelKg);
        barRow("剩余燃油", static_cast<float>(std::clamp(s.fuelKg / 4500.0, 0.02, 1.0)), vbuf, colour(48, 214, 186, 240));
    }
    drawDismissButton(kPanelAero, activePanels);
    ImGui::End();
}

// ============================================================================
// Plugin 6: Own Link 16 / C2 / Mode 5 IFF / PNT & Native Multi-Sensor Fusion
// ============================================================================
void MissionGui::drawCommsPanel(const DockRect& rect, std::uint32_t activePanels) const {
    ImGui::SetNextWindowPos(ImVec2(rect.x, rect.y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(rect.w, rect.h), ImGuiCond_Always);
    if (!beginMandatoryPanel("数据链与融合航迹###comms_panel", ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::End();
        return;
    }
    ImGui::SetWindowFontScale(rect.fontScale);

    int link16Count = 0, c2Count = 0, iffCount = 0;
    bool hasPnt = false, pntJammed = false, pntLocked = false;
    double pntErrM = 0.0;
    for (const auto& l : focusedLinks_) {
        const auto kind = static_cast<ipc::TacticalLinkKindV1>(l.kind);
        if (kind == ipc::TacticalLinkKindV1::Link16Dms) ++link16Count;
        else if (kind == ipc::TacticalLinkKindV1::C2Consent) ++c2Count;
        else if (kind == ipc::TacticalLinkKindV1::Mode5IffInterrogate || kind == ipc::TacticalLinkKindV1::Mode5IffReply) ++iffCount;
        else if (kind == ipc::TacticalLinkKindV1::PntGnssDrift) {
            hasPnt = true;
            pntJammed = (l.flags & ipc::kTactLinkJammed) != 0u;
            pntLocked = (l.flags & ipc::kTactLinkGnssLocked) != 0u;
            pntErrM = static_cast<double>(l.param);
        }
    }
    ImGui::Text("Link 16: %d · C2 授权: %d · IFF: %d", link16Count, c2Count, iffCount);
    if (hasPnt) {
        ImGui::TextColored(pntJammed ? warningText() : tint(0.35f, 0.94f, 0.54f),
                           "PNT 导航: %s · 水平误差 %.1f m",
                           pntJammed ? "GNSS 受干扰" : (pntLocked ? "GNSS 锁定" : "惯导保持"), pntErrM);
    }

    if (fusionValid_) {
        ImGui::Separator();
        ImGui::Text("多源融合航迹: %u / %u", fusion_->count, fusion_->total);
        if (ImGui::BeginTable("fused_dock", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("编号");
            ImGui::TableSetupColumn("来源");
            ImGui::TableSetupColumn("误差 m");
            ImGui::TableSetupColumn("状态");
            ImGui::TableHeadersRow();
            static const char* stNames[] = {"跟踪", "外推", "丢失"};
            for (std::uint32_t i = 0; i < fusion_->count; ++i) {
                const auto& e = fusion_->entities[i];
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::Text("%llu", static_cast<unsigned long long>(e.id));
                ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(e.sourceKinds == 3 ? "雷达+链" : e.sourceKinds == 1 ? "雷达" : "链路");
                ImGui::TableSetColumnIndex(2); ImGui::Text("%.0f", std::sqrt(std::max(0.0, (e.covarianceEcef[0] + e.covarianceEcef[14] + e.covarianceEcef[28]) / 3.0)));
                ImGui::TableSetColumnIndex(3); ImGui::TextUnformatted(e.state < 3 ? stNames[e.state] : "?");
            }
            ImGui::EndTable();
        }
    } else {
        ImGui::TextDisabled("多源融合航迹表仅在实时镜像开启融合时提供");
    }
    drawDismissButton(kPanelComms, activePanels);
    ImGui::End();
}

} // namespace fsim::ui
