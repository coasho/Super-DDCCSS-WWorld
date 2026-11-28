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

bool jammerHitsPoint(const ipc::TacticalJammerLobeWireV1& j, const sim::VehicleState& st) noexcept {
    const double dx = st.positionEcef[0] - j.originEcef[0];
    const double dy = st.positionEcef[1] - j.originEcef[1];
    const double dz = st.positionEcef[2] - j.originEcef[2];
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < 1.0 || dist > std::max(static_cast<double>(j.rangeM), 15000.0) * 1.2) return false;
    if ((j.flags & ipc::kTactJamAllRound) != 0u) return true;
    const double bn = std::sqrt(j.boresightEcef[0] * j.boresightEcef[0] +
                                j.boresightEcef[1] * j.boresightEcef[1] +
                                j.boresightEcef[2] * j.boresightEcef[2]);
    if (bn < 1e-6) return false;
    const double cosA = (dx * j.boresightEcef[0] + dy * j.boresightEcef[1] + dz * j.boresightEcef[2]) / (dist * bn);
    const double halfAng = std::clamp(static_cast<double>(j.halfAngleRad), 0.08, 0.85) + 0.16;
    return cosA >= std::cos(halfAng);
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

ImU32 heatColor(float norm01) {
    // Deep-space scientific Radar DSP colormap (navy -> indigo -> violet -> cyan -> gold -> hot white):
    // keeps thermal noise calm dark navy, renders broadband jamming in distinct deep violet,
    // and makes ground clutter & CFAR Doppler peaks crisp and unmistakable.
    struct Stop { float t, r, g, b; };
    static constexpr Stop kStops[] = {
        {0.00f,   5.0f,  11.0f,  24.0f},
        {0.24f,  18.0f,  34.0f,  78.0f},
        {0.48f,  78.0f,  46.0f, 148.0f},
        {0.70f,  26.0f, 174.0f, 212.0f},
        {0.87f, 248.0f, 188.0f,  46.0f},
        {1.00f, 255.0f, 250.0f, 232.0f},
    };
    const float t = std::clamp(norm01, 0.0f, 1.0f);
    for (int i = 0; i < 5; ++i) {
        if (t <= kStops[i + 1].t) {
            const float u = (t - kStops[i].t) / (kStops[i + 1].t - kStops[i].t);
            const int r = static_cast<int>(kStops[i].r + u * (kStops[i + 1].r - kStops[i].r));
            const int g = static_cast<int>(kStops[i].g + u * (kStops[i + 1].g - kStops[i].g));
            const int b = static_cast<int>(kStops[i].b + u * (kStops[i + 1].b - kStops[i].b));
            return colour(r, g, b, 255);
        }
    }
    return colour(255, 250, 232, 255);
}

double cellHash01(int r, int c, int tick) noexcept {
    std::uint32_t h = static_cast<std::uint32_t>(r) * 73856093u ^
                      static_cast<std::uint32_t>(c) * 19349663u ^
                      static_cast<std::uint32_t>(tick) * 83492791u;
    h ^= h >> 16;
    h *= 0x7feb352du;
    h ^= h >> 15;
    h *= 0x846ca68bu;
    h ^= h >> 16;
    return static_cast<double>(h & 0xFFFFu) / 65535.0;
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
    relevantJammers_.clear();
    focusedEsm_.clear();
    focusedWeapons_.clear();
    focusedLinks_.clear();
    focusedProp_.clear();
    if (!tactical || slot < 0) return;

    bool radarJammed = false;
    for (const auto& r : tactical->radars) {
        if (static_cast<int>(r.slot) == slot) {
            focusedRadars_.push_back(r);
            if ((r.flags & ipc::kTactRadarJammed) != 0u) radarJammed = true;
        }
    }
    for (const auto& j : tactical->jammers) {
        if (static_cast<int>(j.slot) == slot || (radarJammed && hasOwnState_ && jammerHitsPoint(j, ownState_))) {
            relevantJammers_.push_back(j);
        }
    }
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

    // Track closing rate smoothly from native targetRangeM delta for derived R-D visualization
    if (!focusedRadars_.empty() && (focusedRadars_[0].flags & ipc::kTactRadarHasTarget) != 0u) {
        const double rNow = static_cast<double>(focusedRadars_[0].targetRangeM);
        if (prevTargetRangeM_ > 0.0 && simTime_ > prevTargetRangeTimeS_ + 0.02 && simTime_ - prevTargetRangeTimeS_ < 1.5) {
            const double rawClose = (prevTargetRangeM_ - rNow) / (simTime_ - prevTargetRangeTimeS_);
            estClosingMs_ = 0.75 * estClosingMs_ + 0.25 * std::clamp(rawClose, -1200.0, 1200.0);
        }
        prevTargetRangeM_ = rNow;
        prevTargetRangeTimeS_ = simTime_;
    } else {
        prevTargetRangeM_ = -1.0;
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
        prevTargetRangeM_ = -1.0;
        estClosingMs_ = 0.0;
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
    if (slot != slot_) {
        prevTargetRangeM_ = -1.0;
        estClosingMs_ = 0.0;
    }
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

void MissionGui::record(vsg::CommandBuffer&) const {
    if (!mirrored_ && !replayMode_) {
        if (controls_) controls_->rightDockLeftPx.store(0.0f, std::memory_order_relaxed);
        return;
    }

    const float k = uiScale();
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float margin = 10.0f * k;
    const auto preset = static_cast<WorkspacePreset>(
        controls_ ? std::clamp(controls_->workspacePreset.load(std::memory_order_relaxed), 0, kWorkspacePresetCount - 1)
                  : static_cast<int>(WorkspacePreset::AirRadar));
    const std::uint32_t customMask = controls_ ? controls_->customPanelMask.load(std::memory_order_relaxed) : kPanelRadar;
    const std::uint32_t activePanels = panelsForWorkspace(preset, customMask);

    // Build the ordered list of docked right-side plugin panels supported by the currently focused vehicle.
    struct DockSlot {
        std::uint32_t bit = 0u;
        float weight = 1.0f;
    };
    std::vector<DockSlot> docked;
    docked.reserve(6);

    // 1. Airborne Radar is MANDATORY on radar-equipped aircraft and strictly HIDDEN on missiles/helis/GA!
    if (supportsRadar_) {
        docked.push_back({kPanelRadar, 1.25f});
    }
    // 2. Visual Signal Lab (Range-Doppler / Burn-Through SNR / Polar Pattern): radar-equipped aircraft only.
    if ((activePanels & kPanelSignal) != 0u && supportsRadar_) {
        docked.push_back({kPanelSignal, 1.15f});
    }
    // 3. EMSO / EW Spectrum & ESM Bearing / CRLB panel.
    if ((activePanels & kPanelEw) != 0u && (supportsRadar_ || !relevantJammers_.empty() || !focusedEsm_.empty())) {
        docked.push_back({kPanelEw, 0.95f});
    }
    // 4. 5-D Weapon DLZ / LCOS / CCIP & Stores panel.
    if ((activePanels & kPanelWeapon) != 0u && (!focusedWeapons_.empty() || (valid_ && record_->storeCount > 0) || !supportsRadar_)) {
        docked.push_back({kPanelWeapon, 0.95f});
    }
    // 5. 6-DOF Aerodynamics & up to 8-Engine / Rotor Propulsion panel (supported on all vehicles).
    if ((activePanels & kPanelAero) != 0u && hasOwnState_) {
        docked.push_back({kPanelAero, 1.00f});
    }
    // 6. Link 16 / C2 / Mode 5 IFF / PNT & Native Fusion panel.
    if ((activePanels & kPanelComms) != 0u && (!focusedLinks_.empty() || fusionValid_ || mirrored_)) {
        docked.push_back({kPanelComms, 0.95f});
    }

    if (docked.empty()) {
        if (controls_) controls_->rightDockLeftPx.store(0.0f, std::memory_order_relaxed);
        return;
    }

    // Elastic Right-Edge Docking Grid: sits cleanly below the top-right desktop clock (~9.5% height)
    // and auto-divides vertical space cleanly across 1..3 active plugin cards (spills to col 2 if >= 4!).
    const float colW = std::min(390.0f * k, display.x * 0.27f);
    const float gap = 8.0f * k;
    const float topBarY = controls_ ? controls_->topBarBottomPx.load(std::memory_order_relaxed) : 0.0f;
    const float startY = std::max({display.y * 0.095f, topBarY + gap, margin});
    const float availH = std::max(200.0f * k, display.y - margin - startY);

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
        const float maxSingleH = (count == 1u) ? std::min(usableH, 450.0f * k) : usableH;
        float curY = startY;
        for (std::size_t i = 0; i < count; ++i) {
            const float share = (count == 1u) ? maxSingleH : (usableH * (docked[beginIdx + i].weight / totalW));
            const DockRect r{colX, curY, colW, share};
            curY += share + gap;
            switch (docked[beginIdx + i].bit) {
            case kPanelRadar:  drawRadarPanel(r); break;
            case kPanelSignal: drawSignalLabPanel(r); break;
            case kPanelEw:     drawEwPanel(r); break;
            case kPanelWeapon: drawWeaponPanel(r); break;
            case kPanelAero:   drawAeroPanel(r); break;
            case kPanelComms:  drawCommsPanel(r); break;
            default: break;
            }
        }
    };

    dispatchColumn(0u, col1Count, col1X);
    if (col2Count > 0u) dispatchColumn(col1Count, col2Count, col2X);
}

// ============================================================================
// Plugin 1: Airborne Radar B-Scope / PPI (Mandatory on Radar-Equipped Aircraft)
// ============================================================================
void MissionGui::drawRadarPanel(const DockRect& rect) const {
    ImGui::SetNextWindowPos(ImVec2(rect.x, rect.y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(rect.w, rect.h), ImGuiCond_Always);
    char title[128];
    std::snprintf(title, sizeof title, "机载雷达 - %s###radar0", name_.empty() ? "?" : name_.c_str());
    if (!beginMandatoryPanel(title, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::End();
        return;
    }

    if (mirrored_ && valid_ && record_->radarCount > 0) {
        drawLiveRadarBody(record_->radars[0]);
    } else if (!focusedRadars_.empty()) {
        drawReplayRadarBody(focusedRadars_[0]);
    } else {
        drawStandbyRadarBody();
    }
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
    if (jammed && matchedTact->burnThroughRangeM > 100.0f) {
        ImGui::TextColored(cautionText(), "干扰压制 · 烧穿 %.1f km (+%.0f dB)",
                           static_cast<double>(matchedTact->burnThroughRangeM) * 1e-3,
                           static_cast<double>(matchedTact->jammerRatioDb));
    }

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float footerH = 1.4f * ImGui::GetTextLineHeightWithSpacing();
    const float scopeH = std::max(80.0f * k, avail.y - footerH);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 size(avail.x, scopeH);
    ImGui::InvisibleButton("live_scope", size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);

    dl->AddRectFilled(p0, p1, colour(8, 17, 22, 252), 4.0f * k);
    dl->AddRect(p0, p1, colour(58, 96, 108, 145), 4.0f * k, 0, 1.0f);
    const ImU32 grid = colour(36, 78, 74, 205), labelCol = colour(124, 182, 166, 235);
    const double scale = r.rangeScaleM > 0.0f ? static_cast<double>(r.rangeScaleM) : 150e3;
    const double fade = std::max(3.0, 2.0 * static_cast<double>(r.framePeriodS));

    char text[32];
    if (!r.rotodome) {
        const double lim = std::max(1e-3, static_cast<double>(r.azimuthLimitRad));
        auto xOf = [&](double az) { return p0.x + static_cast<float>((std::clamp(az / lim, -1.0, 1.0) + 1.0) * 0.5) * size.x; };
        auto yOf = [&](double rng) { return p1.y - static_cast<float>(std::clamp(rng / scale, 0.0, 1.0)) * size.y; };
        if (r.searching) {
            const float x0 = xOf(r.azimuthCentreRad - r.azimuthHalfWidthRad);
            const float x1 = xOf(r.azimuthCentreRad + r.azimuthHalfWidthRad);
            if (jammed && matchedTact->burnThroughRangeM > 100.0f && matchedTact->burnThroughRangeM < scale) {
                const float yBt = yOf(static_cast<double>(matchedTact->burnThroughRangeM));
                dl->AddRectFilled(ImVec2(x0, yBt), ImVec2(x1, p1.y), colour(18, 56, 66, 175));
                dl->AddRectFilled(ImVec2(x0, p0.y), ImVec2(x1, yBt), colour(64, 22, 54, 110));
            } else {
                dl->AddRectFilled(ImVec2(x0, p0.y), ImVec2(x1, p1.y), colour(18, 52, 62, 155));
            }
        }
        for (int q = 1; q < 4; ++q) {
            const float yy = yOf(scale * q / 4.0);
            dl->AddLine(ImVec2(p0.x, yy), ImVec2(p1.x, yy), grid);
            std::snprintf(text, sizeof text, "%.0f km", scale * q / 4.0 * 1e-3);
            dl->AddText(ImVec2(p0.x + 4.0f * k, yy - ImGui::GetTextLineHeight()), labelCol, text);
        }
        for (int deg = -60; deg <= 60; deg += 30) {
            if (std::fabs(deg * kDeg) > lim) continue;
            dl->AddLine(ImVec2(xOf(deg * kDeg), p0.y), ImVec2(xOf(deg * kDeg), p1.y), grid);
        }
        if (jammed && matchedTact->burnThroughRangeM > 100.0f && matchedTact->burnThroughRangeM < scale) {
            const float yBt = yOf(static_cast<double>(matchedTact->burnThroughRangeM));
            dl->AddLine(ImVec2(p0.x, yBt), ImVec2(p1.x, yBt), colour(255, 206, 52, 255), 2.2f * k);
        }
        if (r.searching) {
            const float bx = xOf(r.beamAzimuthRad);
            dl->AddLine(ImVec2(bx, p0.y), ImVec2(bx, p1.y), colour(102, 240, 186, 210), 1.9f * k);
        }
        for (std::uint32_t i = 0; i < r.recentCount && i < ipc::kMissionDetections; ++i) {
            const double age = simTime_ - r.recent[i].timeS;
            if (age > fade || age < -1.0) continue;
            const float a = static_cast<float>(std::clamp(1.0 - age / fade, 0.15, 1.0));
            const float h = 3.0f * k;
            const float rx = xOf(r.recent[i].azimuthRad), ry = yOf(r.recent[i].rangeM);
            dl->AddRectFilled(ImVec2(rx - h, ry - h), ImVec2(rx + h, ry + h),
                              r.recent[i].rangeRateMs < 0.0f ? colour(96, 245, 146, static_cast<int>(255 * a))
                                                             : colour(255, 206, 84, static_cast<int>(255 * a)));
        }
        for (std::uint32_t i = 0; i < r.trackCount && i < ipc::kMissionTracks; ++i) {
            if (r.tracks[i].rangeM > scale) continue;
            const float tx = xOf(r.tracks[i].azimuthRad), ty = yOf(r.tracks[i].rangeM);
            const float h = 4.8f * k;
            const ImU32 c = colour(112, 224, 255, 255);
            dl->AddRect(ImVec2(tx - h, ty - h), ImVec2(tx + h, ty + h), c, 0.0f, 0, 1.5f * k);
            if (r.tracks[i].flags & ipc::kMissionTrackDesignated) {
                const float d = 1.9f * h;
                const ImVec2 dia[4] = {{tx, ty - d}, {tx + d, ty}, {tx, ty + d}, {tx - d, ty}};
                dl->AddPolyline(dia, 4, colour(255, 210, 72, 255), ImDrawFlags_Closed, 1.7f * k);
            }
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
        if (r.searching) dl->AddLine(c, at(r.beamAzimuthRad, scale), colour(102, 240, 186, 215), 1.9f * k);
        for (std::uint32_t i = 0; i < r.trackCount && i < ipc::kMissionTracks; ++i) {
            if (r.tracks[i].rangeM > scale) continue;
            const ImVec2 pt = at(r.tracks[i].azimuthRad, r.tracks[i].rangeM);
            dl->AddRect(ImVec2(pt.x - 4.5f * k, pt.y - 4.5f * k), ImVec2(pt.x + 4.5f * k, pt.y + 4.5f * k),
                        colour(112, 224, 255, 255), 0.0f, 0, 1.5f * k);
        }
    }
    ImGui::TextDisabled("航迹 %u（暂态 %u） · 点迹 %llu · 虚警 %llu",
                        static_cast<unsigned>(r.trackCount), static_cast<unsigned>(r.tentativeTracks),
                        static_cast<unsigned long long>(r.detections), static_cast<unsigned long long>(r.falseAlarms));
}

void MissionGui::drawReplayRadarBody(const ipc::TacticalRadarVolumeWireV1& r) const {
    const float k = uiScale();
    const bool searching = (r.flags & ipc::kTactRadarSearching) != 0u;
    const bool roto = (r.flags & ipc::kTactRadarRotodome) != 0u;
    const bool elec = (r.flags & ipc::kTactRadarElectronic) != 0u;
    const bool stt = (r.flags & ipc::kTactRadarSttLock) != 0u;
    const bool illum = (r.flags & ipc::kTactRadarIlluminating) != 0u;
    const bool jammed = (r.flags & ipc::kTactRadarJammed) != 0u;
    const bool hasTarget = (r.flags & ipc::kTactRadarHasTarget) != 0u;

    const double scale = r.rangeScaleM > 1000.0f
                             ? static_cast<double>(r.rangeScaleM)
                             : std::max(40000.0, static_cast<double>(r.detectionRangeM));

    if (searching) {
        ImGui::TextColored(stt || illum ? cautionText() : tint(0.35f, 0.94f, 0.54f), "%s", modeName(r.mode));
        ImGui::SameLine();
        ImGui::Text("%s · 量程 %.0f km", modeLong(r.mode), scale * 1e-3);
        if (illum) { ImGui::SameLine(); ImGui::TextColored(warningText(), "照射"); }
        ImGui::TextDisabled("%s · 方位 %+.0f°±%.0f°",
                            roto ? "360° 旋转雷达罩" : elec ? "AESA 相控阵" : "机械扫描",
                            static_cast<double>(r.azimuthCentreRad) / kDeg,
                            static_cast<double>(r.azimuthHalfWidthRad) / kDeg);
    } else {
        ImGui::TextColored(stateColour(2), "待机（未搜索）");
    }
    if (jammed && r.burnThroughRangeM > 100.0f) {
        ImGui::TextColored(cautionText(), "干扰压制 · 烧穿 %.1f km (+%.0f dB)",
                           static_cast<double>(r.burnThroughRangeM) * 1e-3,
                           static_cast<double>(r.jammerRatioDb));
    }

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float footerH = 1.4f * ImGui::GetTextLineHeightWithSpacing();
    const float scopeH = std::max(80.0f * k, avail.y - footerH);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 size(avail.x, scopeH);
    ImGui::InvisibleButton("replay_scope", size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);

    dl->AddRectFilled(p0, p1, colour(8, 17, 22, 252), 4.0f * k);
    dl->AddRect(p0, p1, colour(58, 96, 108, 150), 4.0f * k, 0, 1.0f);
    const ImU32 grid = colour(36, 78, 74, 210), labelCol = colour(128, 184, 168, 240);

    bool tgtValid = false;
    double tgtAz = 0.0, tgtEl = 0.0, tgtRange = 0.0;
    if (hasTarget && hasOwnState_) {
        bodyAzEl(ownState_, r.targetEcef, tgtAz, tgtEl, tgtRange);
        if (r.targetRangeM > 1.0f) tgtRange = static_cast<double>(r.targetRangeM);
        tgtValid = (tgtRange > 1.0);
    }

    char text[48];
    if (!roto) {
        const double lim = std::max(60.0 * kDeg, std::abs(static_cast<double>(r.azimuthCentreRad)) +
                                                 static_cast<double>(r.azimuthHalfWidthRad) + 4.0 * kDeg);
        auto xOf = [&](double az) { return p0.x + static_cast<float>((std::clamp(az / lim, -1.0, 1.0) + 1.0) * 0.5) * size.x; };
        auto yOf = [&](double rng) { return p1.y - static_cast<float>(std::clamp(rng / scale, 0.0, 1.0)) * size.y; };

        if (searching) {
            const float x0 = xOf(static_cast<double>(r.azimuthCentreRad) - static_cast<double>(r.azimuthHalfWidthRad));
            const float x1 = xOf(static_cast<double>(r.azimuthCentreRad) + static_cast<double>(r.azimuthHalfWidthRad));
            if (jammed && r.burnThroughRangeM > 100.0f && r.burnThroughRangeM < scale) {
                const float yBt = yOf(static_cast<double>(r.burnThroughRangeM));
                dl->AddRectFilled(ImVec2(x0, yBt), ImVec2(x1, p1.y), colour(18, 56, 66, 175));
                dl->AddRectFilled(ImVec2(x0, p0.y), ImVec2(x1, yBt), colour(64, 22, 54, 110));
            } else {
                dl->AddRectFilled(ImVec2(x0, p0.y), ImVec2(x1, p1.y), colour(18, 52, 62, 160));
            }
        }
        for (int q = 1; q < 4; ++q) {
            const float yy = yOf(scale * q / 4.0);
            dl->AddLine(ImVec2(p0.x, yy), ImVec2(p1.x, yy), grid);
            std::snprintf(text, sizeof text, "%.0f km", scale * q / 4.0 * 1e-3);
            dl->AddText(ImVec2(p0.x + 4.0f * k, yy - ImGui::GetTextLineHeight()), labelCol, text);
        }
        for (int deg = -60; deg <= 60; deg += 30) {
            if (std::fabs(deg * kDeg) > lim + 1e-3) continue;
            dl->AddLine(ImVec2(xOf(deg * kDeg), p0.y), ImVec2(xOf(deg * kDeg), p1.y), grid);
        }
        if (jammed && r.burnThroughRangeM > 100.0f && r.burnThroughRangeM < scale) {
            const float yBt = yOf(static_cast<double>(r.burnThroughRangeM));
            dl->AddLine(ImVec2(p0.x, yBt), ImVec2(p1.x, yBt), colour(255, 206, 52, 255), 2.2f * k);
            std::snprintf(text, sizeof text, "烧穿 %.1f km", static_cast<double>(r.burnThroughRangeM) * 1e-3);
            const ImVec2 tsz = ImGui::CalcTextSize(text);
            dl->AddText(ImVec2(p1.x - tsz.x - 6.0f * k, std::max(p0.y + 2.0f * k, yBt - tsz.y - 2.0f * k)),
                        colour(255, 218, 92, 255), text);
        }
        if (searching) {
            const float bx = xOf(static_cast<double>(r.beamAzimuthRad));
            dl->AddLine(ImVec2(bx, p0.y), ImVec2(bx, p1.y), colour(102, 240, 186, 215), 2.0f * k);
        }
        if (tgtValid && std::abs(tgtAz) <= lim && tgtRange <= scale) {
            const float tx = xOf(tgtAz), ty = yOf(tgtRange);
            const float h = 5.0f * k;
            const ImU32 tCol = (stt || illum) ? colour(255, 212, 74, 255) : colour(112, 224, 255, 255);
            dl->AddRect(ImVec2(tx - h, ty - h), ImVec2(tx + h, ty + h), tCol, 0.0f, 0, 1.7f * k);
            if (stt || illum) {
                const float d = 1.9f * h;
                const ImVec2 dia[4] = {{tx, ty - d}, {tx + d, ty}, {tx, ty + d}, {tx - d, ty}};
                dl->AddPolyline(dia, 4, colour(255, 212, 74, 255), ImDrawFlags_Closed, 1.7f * k);
            }
            std::snprintf(text, sizeof text, "%.1f km", tgtRange * 1e-3);
            dl->AddText(ImVec2(tx + h + 4.0f * k, ty - ImGui::GetTextLineHeight() * 0.5f), tCol, text);
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
            dl->AddCircle(c, static_cast<float>(r.burnThroughRangeM / scale) * radius, colour(255, 206, 52, 255), 56, 2.2f * k);
        }
        if (searching) dl->AddLine(c, at(static_cast<double>(r.beamAzimuthRad), scale), colour(102, 240, 186, 220), 2.0f * k);
        if (tgtValid && tgtRange <= scale) {
            const ImVec2 q = at(tgtAz, tgtRange);
            dl->AddRect(ImVec2(q.x - 5.0f * k, q.y - 5.0f * k), ImVec2(q.x + 5.0f * k, q.y + 5.0f * k),
                        colour(112, 224, 255, 255), 0.0f, 0, 1.7f * k);
        }
    }

    ImGui::TextDisabled("标称探测 %.0f km · 波束 %+.1f°/%+.1f°",
                        static_cast<double>(r.detectionRangeM) * 1e-3,
                        static_cast<double>(r.beamAzimuthRad) / kDeg,
                        static_cast<double>(r.beamElevationRad) / kDeg);
}

void MissionGui::drawStandbyRadarBody() const {
    const float k = uiScale();
    ImGui::TextColored(stateColour(2), "待机 / 雷达静默");
    ImGui::TextDisabled("当前未发射雷达搜索波束");
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float h = std::max(80.0f * k, avail.y - 4.0f * k);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 size(avail.x, h);
    ImGui::InvisibleButton("standby_scope", size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);
    dl->AddRectFilled(p0, p1, colour(8, 17, 22, 245), 4.0f * k);
    dl->AddRect(p0, p1, colour(58, 96, 108, 120), 4.0f * k, 0, 1.0f);
    const ImU32 grid = colour(30, 62, 62, 165);
    for (int q = 1; q < 4; ++q) {
        dl->AddLine(ImVec2(p0.x, p1.y - size.y * static_cast<float>(q) / 4.0f),
                    ImVec2(p1.x, p1.y - size.y * static_cast<float>(q) / 4.0f), grid);
        dl->AddLine(ImVec2(p0.x + size.x * static_cast<float>(q) / 4.0f, p0.y),
                    ImVec2(p0.x + size.x * static_cast<float>(q) / 4.0f, p1.y), grid);
    }
    const char* msg = "雷达静默 / 待机";
    const ImVec2 msz = ImGui::CalcTextSize(msg);
    dl->AddText(ImVec2(p0.x + (size.x - msz.x) * 0.5f, p0.y + (size.y - msz.y) * 0.5f), colour(126, 164, 172, 215), msg);
}

// ============================================================================
// Plugin 2: Visual Radar Signal Lab (Range-Doppler / Burn-Through SNR / Polar)
// ============================================================================
void MissionGui::drawSignalLabPanel(const DockRect& rect) const {
    const float k = uiScale();
    ImGui::SetNextWindowPos(ImVec2(rect.x, rect.y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(rect.w, rect.h), ImGuiCond_Always);
    if (!beginMandatoryPanel("雷达底层可视化###radar_signal_lab", ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::End();
        return;
    }

    int tab = controls_ ? std::clamp(controls_->radarSignalTab.load(std::memory_order_relaxed), 0, 2) : 0;
    const char* tabs[3] = {"距离-多普勒", "烧穿曲线", "天线方向图"};
    for (int i = 0; i < 3; ++i) {
        if (i > 0) ImGui::SameLine(0.0f, 5.0f * k);
        if (tab == i) {
            ImGui::PushStyleColor(ImGuiCol_Button, tint(0.14f, 0.44f, 0.72f, 0.96f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, tint(0.18f, 0.52f, 0.82f, 1.0f));
        }
        if (ImGui::SmallButton(tabs[i])) {
            tab = i;
            if (controls_) controls_->radarSignalTab.store(tab, std::memory_order_relaxed);
        }
        if (tab == i) ImGui::PopStyleColor(2);
    }

    const ipc::TacticalRadarVolumeWireV1* tact = focusedRadars_.empty() ? nullptr : &focusedRadars_[0];
    const ipc::MissionRadar* live = (mirrored_ && valid_ && record_->radarCount > 0) ? &record_->radars[0] : nullptr;

    if (tab == 0) {
        if (live && record_->rangeDoppler.radar < record_->radarCount &&
            record_->radars[record_->rangeDoppler.radar].signalLevel &&
            record_->rangeDoppler.rows > 0 && record_->rangeDoppler.columns > 0) {
            drawNativeRangeDoppler(record_->rangeDoppler);
        } else {
            drawDerivedRangeDoppler(tact, live);
        }
    } else if (tab == 1) {
        drawBurnThroughCurve(tact, live);
    } else {
        drawAntennaPolar(tact, live);
    }
    ImGui::End();
}

void MissionGui::drawNativeRangeDoppler(const ipc::MissionRangeDoppler& rd) const {
    const float k = uiScale();
    const int rows = rd.rows, cols = rd.columns;
    ImGui::TextDisabled("原生信号级 R-D · PRF %.1f kHz · %.1f km × %.0f m/s",
                        static_cast<double>(rd.prfHz) * 1e-3,
                        static_cast<double>(rd.ambiguousRangeM) * 1e-3,
                        static_cast<double>(rd.ambiguousRateMs));
    const float axisW = ImGui::CalcTextSize("00").x + 6.0f * k;
    const float axisH = ImGui::GetTextLineHeight() + 3.0f * k;
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 size(std::max(60.0f * k, avail.x - axisW), std::max(60.0f * k, avail.y - axisH));
    ImGui::InvisibleButton("rd_native", ImVec2(size.x + axisW, size.y + axisH));
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p0(origin.x + axisW, origin.y), p1(p0.x + size.x, p0.y + size.y);
    dl->AddRectFilled(p0, p1, colour(4, 9, 15, 255));

    const float cw = size.x / static_cast<float>(cols), ch = size.y / static_cast<float>(rows);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const int q = rd.db[static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) + static_cast<std::size_t>(c)];
            if (q < 12) continue;
            const float x = p0.x + static_cast<float>(c) * cw;
            const float y = p1.y - static_cast<float>(r + 1) * ch;
            dl->AddRectFilled(ImVec2(x, y), ImVec2(x + cw + 0.5f, y + ch + 0.5f), heatColor(static_cast<float>(q) / 160.0f));
        }
    }
    const int filters = cols * rd.columnBin;
    const double va = std::max(1.0, static_cast<double>(rd.ambiguousRateMs));
    auto xAt = [&](double closing) {
        double f = closing / va;
        f -= std::floor(f + 0.5);
        return p0.x + static_cast<float>((f + 0.5) * static_cast<double>(filters) / static_cast<double>(rd.columnBin)) * cw;
    };
    const ImU32 notch = colour(90, 156, 255, 230);
    if (std::isfinite(rd.mainBeamRateMs) && rd.notchMs > 0.0f) {
        const double centre = -static_cast<double>(rd.mainBeamRateMs);
        for (double edge : {centre - rd.notchMs, centre + rd.notchMs})
            dl->AddLine(ImVec2(xAt(edge), p0.y), ImVec2(xAt(edge), p1.y), notch, 1.3f * k);
    }
    for (std::uint32_t i = 0; i < rd.peakCount && i < ipc::kRangeDopplerPeaks; ++i) {
        const ImVec2 c(p0.x + (static_cast<float>(rd.peakColumn[i]) + 0.5f) * cw,
                       p1.y - (static_cast<float>(rd.peakRow[i]) + 0.5f) * ch);
        dl->AddCircle(c, std::max(4.5f * k, 0.8f * std::max(cw, ch)), colour(255, 86, 204, 255), 16, 1.6f * k);
    }
    dl->AddRect(p0, p1, colour(56, 86, 114, 150));
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.0f", static_cast<double>(rd.ambiguousRangeM) * 1e-3);
    dl->AddText(ImVec2(origin.x, p0.y), colour(140, 164, 184, 235), buf);
    dl->AddText(ImVec2(origin.x, p1.y - ImGui::GetTextLineHeight()), colour(140, 164, 184, 235), "0");
    std::snprintf(buf, sizeof buf, "%+.0f", -va * 0.5);
    dl->AddText(ImVec2(p0.x, p1.y + 2.0f), colour(140, 164, 184, 235), buf);
    dl->AddText(ImVec2(p0.x + size.x * 0.44f, p1.y + 2.0f), colour(140, 164, 184, 235), "0 m/s");
    std::snprintf(buf, sizeof buf, "%+.0f", va * 0.5);
    dl->AddText(ImVec2(p1.x - ImGui::CalcTextSize(buf).x, p1.y + 2.0f), colour(140, 164, 184, 235), buf);
}

void MissionGui::drawDerivedRangeDoppler(const ipc::TacticalRadarVolumeWireV1* tact, const ipc::MissionRadar* live) const {
    const float k = uiScale();
    if (!tact && (!live || !live->searching)) {
        ImGui::TextDisabled("雷达静默中，无距离-多普勒回波");
        return;
    }
    const double prfHz = 14500.0;
    const double rUaM = 299792458.0 / (2.0 * prfHz); // ~10.34 km unambiguous range
    const double freqHz = (live && live->frequencyHz > 1e8) ? live->frequencyHz : 9.6e9;
    const double vUaMs = (299792458.0 / freqHz) * prfHz * 0.5; // ~226 m/s unambiguous Doppler
    const double jamDb = tact ? static_cast<double>(tact->jammerRatioDb) : 0.0;
    const bool jammed = tact && (tact->flags & ipc::kTactRadarJammed) != 0u;
    const bool hasTgt = tact && (tact->flags & ipc::kTactRadarHasTarget) != 0u;
    const double tgtRangeM = hasTgt ? static_cast<double>(tact->targetRangeM) : 0.0;
    const double r0M = tact ? static_cast<double>(tact->detectionRangeM) : (live ? static_cast<double>(live->rangeScaleM) : 80e3);

    ImGui::TextDisabled("物理推导 R-D · PRF %.1f kHz · 干扰 +%.0f dB", prfHz * 1e-3, jamDb);

    const float axisW = ImGui::CalcTextSize("10k").x + 6.0f * k;
    const float axisH = ImGui::GetTextLineHeight() + 4.0f * k;
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 size(std::max(60.0f * k, avail.x - axisW), std::max(60.0f * k, avail.y - axisH));
    ImGui::InvisibleButton("rd_derived", ImVec2(size.x + axisW, size.y + axisH));
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p0(origin.x + axisW, origin.y), p1(p0.x + size.x, p0.y + size.y);
    dl->AddRectFilled(p0, p1, colour(4, 9, 15, 255));

    constexpr int kCols = 48;
    constexpr int kRows = 28;
    const float cw = size.x / static_cast<float>(kCols);
    const float ch = size.y / static_cast<float>(kRows);

    // Fold target range & closing velocity into unambiguous R-D cell
    double foldR = hasTgt ? std::fmod(tgtRangeM, rUaM) / rUaM : -1.0;
    double foldV = estClosingMs_ / vUaMs;
    foldV -= std::floor(foldV + 0.5); // [-0.5, 0.5)
    const double tgtColF = (foldV + 0.5) * static_cast<double>(kCols);
    const double tgtRowF = foldR * static_cast<double>(kRows);
    const double ownSpd = hasOwnState_ ? ownState_.airspeedTrueMs : 250.0;
    double mbFold = (-ownSpd) / vUaMs;
    mbFold -= std::floor(mbFold + 0.5);
    const double mbColF = (mbFold + 0.5) * static_cast<double>(kCols);
    const double altFold = hasOwnState_ ? (std::fmod(std::max(ownState_.altitudeAglM, 0.0), rUaM) / rUaM) * static_cast<double>(kRows) : 6.0;

    // Target signal strength over thermal noise from radar equation (40*log10(R0 / R))
    const double tgtSnrDb = (hasTgt && tgtRangeM > 50.0 && r0M > 100.0)
                                ? std::clamp(40.0 * std::log10(r0M / tgtRangeM) + 14.0, 0.0, 56.0)
                                : 0.0;
    const double jamFloorNorm = jammed ? std::clamp(jamDb / 115.0, 0.14, 0.34) : 0.0;
    const int tickBucket = static_cast<int>(simTime_ * 6.0);

    for (int r = 0; r < kRows; ++r) {
        for (int c = 0; c < kCols; ++c) {
            const double hash = cellHash01(r, c, tickBucket);
            double p = 0.04 + 0.07 * hash + jamFloorNorm * (0.76 + 0.34 * hash);
            // Main-beam ground clutter ridge (Doppler width ~2.2 bins)
            double dc = std::abs(static_cast<double>(c) + 0.5 - mbColF);
            dc = std::min(dc, static_cast<double>(kCols) - dc);
            p += 0.44 * std::exp(-(dc * dc) / 3.2);
            // Altitude-line specular clutter
            const double drAlt = static_cast<double>(r) + 0.5 - altFold;
            p += 0.30 * std::exp(-(drAlt * drAlt) / 1.8) * std::exp(-(dc * dc) / 28.0);
            // Real target Doppler echo peak
            if (hasTgt) {
                double dcv = std::abs(static_cast<double>(c) + 0.5 - tgtColF);
                dcv = std::min(dcv, static_cast<double>(kCols) - dcv);
                const double drv = static_cast<double>(r) + 0.5 - tgtRowF;
                p += (0.28 + tgtSnrDb / 64.0) * std::exp(-(dcv * dcv + drv * drv) / 2.2);
            }
            if (p < 0.06) continue;
            const float x = p0.x + static_cast<float>(c) * cw;
            const float y = p1.y - static_cast<float>(r + 1) * ch;
            dl->AddRectFilled(ImVec2(x, y), ImVec2(x + cw + 0.5f, y + ch + 0.5f), heatColor(static_cast<float>(p)));
        }
    }

    // Main-beam clutter notch lines (cyan-blue)
    const ImU32 notch = colour(92, 178, 255, 228);
    for (double off : {-2.6, 2.6}) {
        double nc = std::fmod(mbColF + off + kCols, static_cast<double>(kCols));
        const float nx = p0.x + static_cast<float>(nc) * cw;
        dl->AddLine(ImVec2(nx, p0.y), ImVec2(nx, p1.y), notch, 1.2f * k);
    }
    // CFAR detection circle around target peak when above jammer + threshold
    if (hasTgt && tgtSnrDb > jamDb + 4.0) {
        const ImVec2 pt(p0.x + static_cast<float>(tgtColF) * cw, p1.y - static_cast<float>(tgtRowF) * ch);
        dl->AddCircle(pt, 6.0f * k, colour(255, 86, 204, 255), 18, 1.7f * k);
    }

    dl->AddRect(p0, p1, colour(56, 86, 114, 155));
    const ImU32 axCol = colour(142, 168, 188, 240);
    dl->AddText(ImVec2(origin.x, p0.y), axCol, "10k");
    dl->AddText(ImVec2(origin.x, p0.y + size.y * 0.45f), axCol, "5k");
    dl->AddText(ImVec2(origin.x, p1.y - ImGui::GetTextLineHeight()), axCol, "0");
    char vbuf[32];
    std::snprintf(vbuf, sizeof vbuf, "%+.0f", -vUaMs * 0.5);
    dl->AddText(ImVec2(p0.x, p1.y + 2.0f), axCol, vbuf);
    dl->AddText(ImVec2(p0.x + size.x * 0.42f, p1.y + 2.0f), axCol, "0 m/s");
    std::snprintf(vbuf, sizeof vbuf, "%+.0f", vUaMs * 0.5);
    dl->AddText(ImVec2(p1.x - ImGui::CalcTextSize(vbuf).x, p1.y + 2.0f), axCol, vbuf);
}

void MissionGui::drawBurnThroughCurve(const ipc::TacticalRadarVolumeWireV1* tact, const ipc::MissionRadar* live) const {
    const float k = uiScale();
    const double r0Km = tact && tact->detectionRangeM > 100.0f ? static_cast<double>(tact->detectionRangeM) * 1e-3
                      : (live && live->rangeScaleM > 100.0f ? static_cast<double>(live->rangeScaleM) * 1e-3 : 80.0);
    const double scaleKm = std::max(r0Km * 1.12, tact && tact->rangeScaleM > 1000.0f ? static_cast<double>(tact->rangeScaleM) * 1e-3 : 60.0);
    const double jamDb = tact ? static_cast<double>(tact->jammerRatioDb) : 0.0;
    const bool jammed = tact && (tact->flags & ipc::kTactRadarJammed) != 0u;
    const double btKm = jammed && tact->burnThroughRangeM > 100.0f
                            ? static_cast<double>(tact->burnThroughRangeM) * 1e-3
                            : r0Km * std::pow(10.0, -jamDb / 40.0);
    const bool hasTgt = tact && (tact->flags & ipc::kTactRadarHasTarget) != 0u;
    const double tgtKm = hasTgt ? static_cast<double>(tact->targetRangeM) * 1e-3 : -1.0;

    ImGui::TextDisabled("烧穿方程 Rbt = R0·(1+J0/N0)^(-1/4)  ·  R0=%.0f km", r0Km);

    const float axisW = ImGui::CalcTextSize("+80").x + 6.0f * k;
    const float axisH = ImGui::GetTextLineHeight() + 4.0f * k;
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 size(std::max(60.0f * k, avail.x - axisW), std::max(60.0f * k, avail.y - axisH));
    ImGui::InvisibleButton("snr_curve", ImVec2(size.x + axisW, size.y + axisH));
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p0(origin.x + axisW, origin.y), p1(p0.x + size.x, p0.y + size.y);
    dl->AddRectFilled(p0, p1, colour(6, 12, 20, 252), 4.0f * k);
    dl->AddRect(p0, p1, colour(56, 86, 114, 155), 4.0f * k);

    constexpr double dbMin = -10.0, dbMax = 80.0;
    auto xOf = [&](double km) { return p0.x + static_cast<float>(std::clamp(km / scaleKm, 0.0, 1.0)) * size.x; };
    auto yOf = [&](double db) { return p1.y - static_cast<float>(std::clamp((db - dbMin) / (dbMax - dbMin), 0.0, 1.0)) * size.y; };

    const ImU32 grid = colour(34, 52, 72, 165), axCol = colour(142, 168, 188, 240);
    for (int db : {0, 20, 40, 60}) {
        const float yy = yOf(static_cast<double>(db));
        dl->AddLine(ImVec2(p0.x, yy), ImVec2(p1.x, yy), grid);
        char b[16];
        std::snprintf(b, sizeof b, "%+d", db);
        dl->AddText(ImVec2(origin.x, yy - ImGui::GetTextLineHeight() * 0.5f), axCol, b);
    }

    // Thermal noise floor N0 (0 dB)
    const float yN0 = yOf(0.0);
    dl->AddLine(ImVec2(p0.x, yN0), ImVec2(p1.x, yN0), colour(56, 184, 255, 200), 1.3f * k);
    dl->AddText(ImVec2(p1.x - ImGui::CalcTextSize("N0").x - 4.0f * k, yN0 - ImGui::GetTextLineHeight()), colour(110, 206, 255, 240), "N0");

    // Jammer raised floor N0 + J0 (+jamDb dB)
    const float yJ0 = yOf(jamDb);
    if (jamDb > 0.5) {
        dl->AddRectFilled(ImVec2(p0.x, yJ0), ImVec2(p1.x, yN0), colour(168, 72, 240, 44));
        dl->AddLine(ImVec2(p0.x, yJ0), ImVec2(p1.x, yJ0), colour(198, 96, 252, 235), 1.8f * k);
        char jb[32];
        std::snprintf(jb, sizeof jb, "N0+J0 (+%.0fdB)", jamDb);
        const ImVec2 jsz = ImGui::CalcTextSize(jb);
        dl->AddText(ImVec2(p1.x - jsz.x - 4.0f * k, std::max(p0.y + 2.0f, yJ0 - jsz.y - 2.0f)), colour(216, 152, 255, 250), jb);
    }

    // Emerald R^-4 Target Echo Curve: SNR_dB(R) = 40 * log10(R0 / R)
    constexpr int kSteps = 60;
    ImVec2 prevPt;
    for (int i = 0; i <= kSteps; ++i) {
        const double km = std::max(0.5, scaleKm * (static_cast<double>(i) / static_cast<double>(kSteps)));
        const double snrDb = 40.0 * std::log10(r0Km / km);
        const ImVec2 pt(xOf(km), yOf(snrDb));
        if (i > 0) dl->AddLine(prevPt, pt, colour(50, 224, 146, 245), 2.1f * k);
        prevPt = pt;
    }

    // Exact Gold Burn-Through Intersection Dot (Rbt, jamDb)
    if (btKm > 0.2 && btKm <= scaleKm) {
        const float bx = xOf(btKm);
        dl->AddLine(ImVec2(bx, yJ0), ImVec2(bx, p1.y), colour(255, 206, 52, 165), 1.2f * k);
        dl->AddCircleFilled(ImVec2(bx, yJ0), 4.5f * k, colour(255, 206, 52, 255));
        char btb[32];
        std::snprintf(btb, sizeof btb, "烧穿 %.1fkm", btKm);
        dl->AddText(ImVec2(std::min(bx + 6.0f * k, p1.x - ImGui::CalcTextSize(btb).x - 4.0f),
                           std::max(p0.y + 2.0f, yJ0 - ImGui::GetTextLineHeight() - 2.0f)),
                    colour(255, 218, 92, 255), btb);
    }

    // Live Tracked Target Position on the R^-4 Curve
    if (hasTgt && tgtKm > 0.2 && tgtKm <= scaleKm) {
        const float tx = xOf(tgtKm);
        const float ty = yOf(40.0 * std::log10(r0Km / tgtKm));
        const float r = 4.8f * k;
        const ImVec2 dia[4] = {{tx, ty - r}, {tx + r, ty}, {tx, ty + r}, {tx - r, ty}};
        dl->AddPolyline(dia, 4, colour(86, 218, 255, 255), ImDrawFlags_Closed, 1.8f * k);
    }

    dl->AddText(ImVec2(p0.x, p1.y + 2.0f), axCol, "0 km");
    char rb[24];
    std::snprintf(rb, sizeof rb, "%.0f km", scaleKm);
    dl->AddText(ImVec2(p1.x - ImGui::CalcTextSize(rb).x, p1.y + 2.0f), axCol, rb);
}

void MissionGui::drawAntennaPolar(const ipc::TacticalRadarVolumeWireV1* tact, const ipc::MissionRadar* live) const {
    const float k = uiScale();
    const double beamAz = tact ? static_cast<double>(tact->beamAzimuthRad)
                               : (live ? static_cast<double>(live->beamAzimuthRad) : 0.0);
    const double azC = tact ? static_cast<double>(tact->azimuthCentreRad)
                            : (live ? static_cast<double>(live->azimuthCentreRad) : 0.0);
    const double azH = tact ? static_cast<double>(tact->azimuthHalfWidthRad)
                            : (live ? static_cast<double>(live->azimuthHalfWidthRad) : 0.52);
    const bool elec = tact ? ((tact->flags & ipc::kTactRadarElectronic) != 0u) : (live && live->electronic);

    ImGui::TextDisabled("%s方向图 · 波束 %+.1f° · 扫描 ±%.0f°",
                        elec ? "AESA 相控阵" : "机械扫描", beamAz / kDeg, azH / kDeg);

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 size(avail.x, std::max(80.0f * k, avail.y - 4.0f * k));
    ImGui::InvisibleButton("ant_polar", size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);
    dl->AddRectFilled(p0, p1, colour(6, 12, 20, 252), 4.0f * k);
    dl->AddRect(p0, p1, colour(56, 86, 114, 155), 4.0f * k);

    const ImVec2 c(p0.x + size.x * 0.5f, p1.y - 14.0f * k);
    const float R = std::min(size.x * 0.44f, size.y - 24.0f * k);
    const ImU32 grid = colour(36, 58, 78, 170);

    // Semicircular gain rings (-30 dB, -15 dB, 0 dB peak)
    for (float f : {0.33f, 0.66f, 1.0f}) {
        dl->PathArcTo(c, R * f, static_cast<float>(kPi), static_cast<float>(2.0 * kPi), 40);
        dl->PathStroke(grid, 0, 1.0f);
    }
    for (int deg = -90; deg <= 90; deg += 30) {
        const float rad = static_cast<float>(deg * kDeg);
        dl->AddLine(c, ImVec2(c.x + std::sin(rad) * R, c.y - std::cos(rad) * R), grid);
    }

    // Shaded scanned azimuth limits
    const float aLo = static_cast<float>(std::clamp(azC - azH, -kPi * 0.5, kPi * 0.5));
    const float aHi = static_cast<float>(std::clamp(azC + azH, -kPi * 0.5, kPi * 0.5));
    dl->AddLine(c, ImVec2(c.x + std::sin(aLo) * R, c.y - std::cos(aLo) * R), colour(56, 184, 255, 110), 1.2f * k);
    dl->AddLine(c, ImVec2(c.x + std::sin(aHi) * R, c.y - std::cos(aHi) * R), colour(56, 184, 255, 110), 1.2f * k);

    // Sinc/Taylor mainlobe + sidelobe polar curve steered to live beamAz
    ImVec2 prevPt = c;
    const double beamWidthFactor = elec ? 16.0 : 12.5;
    for (int deg = -90; deg <= 90; deg += 2) {
        const double rad = static_cast<double>(deg) * kDeg;
        const double u = (rad - beamAz) * beamWidthFactor;
        const double sinc = std::abs(u) < 1e-4 ? 1.0 : std::abs(std::sin(u) / u);
        // Phased array scan loss cos^1.2(theta)
        const double scanLoss = elec ? std::pow(std::max(0.25, std::cos(rad)), 1.1) : 1.0;
        const float gainNorm = static_cast<float>(std::clamp(0.10 + 0.90 * std::pow(sinc, 0.58) * scanLoss, 0.08, 1.0));
        const ImVec2 pt(c.x + static_cast<float>(std::sin(rad)) * R * gainNorm,
                        c.y - static_cast<float>(std::cos(rad)) * R * gainNorm);
        if (deg > -90) dl->AddLine(prevPt, pt, colour(62, 198, 255, 245), 1.8f * k);
        prevPt = pt;
    }

    // Live incoming jammer bearing vectors (violet) showing mainlobe vs sidelobe entry!
    if (hasOwnState_) {
        for (const auto& j : relevantJammers_) {
            if (static_cast<int>(j.slot) == slot_) continue;
            double jAz = 0.0, jEl = 0.0, jDist = 0.0;
            bodyAzEl(ownState_, j.originEcef, jAz, jEl, jDist);
            if (std::abs(jAz) <= kPi * 0.52) {
                const ImVec2 tip(c.x + static_cast<float>(std::sin(jAz)) * R * 1.02f,
                                 c.y - static_cast<float>(std::cos(jAz)) * R * 1.02f);
                dl->AddLine(c, tip, colour(204, 102, 255, 245), 2.1f * k);
                dl->AddCircleFilled(tip, 3.8f * k, colour(204, 102, 255, 255));
                char jb[28];
                std::snprintf(jb, sizeof jb, "干扰 %+.0f°", jAz / kDeg);
                dl->AddText(ImVec2(std::clamp(tip.x + 4.0f * k, p0.x + 4.0f, p1.x - ImGui::CalcTextSize(jb).x - 4.0f),
                                   std::max(p0.y + 3.0f, tip.y - ImGui::GetTextLineHeight() * 0.5f)),
                            colour(220, 156, 255, 255), jb);
            }
        }
    }
}

// ============================================================================
// Plugin 3: EMSO / EW Spectrum & ESM Bearing / CRLB Geolocation
// ============================================================================
void MissionGui::drawEwPanel(const DockRect& rect) const {
    const float k = uiScale();
    ImGui::SetNextWindowPos(ImVec2(rect.x, rect.y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(rect.w, rect.h), ImGuiCond_Always);
    if (!beginMandatoryPanel("电子战频谱与侦察###ew_panel", ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::End();
        return;
    }

    // Mini RF spectrum occupancy bar (0..18 GHz)
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float chartH = std::clamp(avail.y * 0.42f, 48.0f * k, 96.0f * k);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 p1(p0.x + avail.x, p0.y + chartH);
    ImGui::InvisibleButton("ew_spec", ImVec2(avail.x, chartH));
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, p1, colour(6, 12, 20, 248), 4.0f * k);
    dl->AddRect(p0, p1, colour(56, 86, 114, 145), 4.0f * k);

    auto xFreq = [&](double ghz) { return p0.x + static_cast<float>(std::clamp(ghz / 18.0, 0.0, 1.0)) * avail.x; };
    for (int g : {2, 6, 10, 14}) {
        const float xx = xFreq(static_cast<double>(g));
        dl->AddLine(ImVec2(xx, p0.y), ImVec2(xx, p1.y), colour(34, 52, 72, 160));
    }
    // Own radar X-band passband bar
    if (!focusedRadars_.empty()) {
        const float rx0 = xFreq(9.45), rx1 = xFreq(9.95);
        dl->AddRectFilled(ImVec2(rx0, p0.y + chartH * 0.22f), ImVec2(rx1, p1.y - 1.0f), colour(52, 182, 255, 115));
        dl->AddRect(ImVec2(rx0, p0.y + chartH * 0.22f), ImVec2(rx1, p1.y - 1.0f), colour(72, 200, 255, 240));
    }
    // Active jammer bands
    for (const auto& j : relevantJammers_) {
        const double bwGhz = std::max(0.25, static_cast<double>(j.bandwidthMhz) * 1e-3);
        const float jx0 = xFreq(static_cast<double>(j.frequencyGhz) - bwGhz * 0.5);
        const float jx1 = xFreq(static_cast<double>(j.frequencyGhz) + bwGhz * 0.5);
        const float hNorm = static_cast<float>(std::clamp((static_cast<double>(j.erpDbw) - 10.0) / 60.0, 0.2, 0.92));
        const ImU32 colFill = (j.flags & ipc::kTactJamComms) ? colour(245, 152, 42, 100) : colour(192, 86, 250, 105);
        const ImU32 colEdge = (j.flags & ipc::kTactJamComms) ? colour(255, 178, 68, 240) : colour(216, 128, 255, 245);
        dl->AddRectFilled(ImVec2(jx0, p1.y - chartH * hNorm), ImVec2(std::max(jx1, jx0 + 6.0f * k), p1.y - 1.0f), colFill);
        dl->AddRect(ImVec2(jx0, p1.y - chartH * hNorm), ImVec2(std::max(jx1, jx0 + 6.0f * k), p1.y - 1.0f), colEdge);
    }
    dl->AddText(ImVec2(p0.x + 4.0f * k, p0.y + 2.0f), colour(142, 168, 188, 235), "0..18 GHz 频谱（蓝=雷达 紫=干扰）");

    if (!relevantJammers_.empty()) {
        for (const auto& j : relevantJammers_) {
            const bool isOwn = (static_cast<int>(j.slot) == slot_);
            ImGui::TextColored(tint(0.84f, 0.56f, 1.0f), "%s · %.1f GHz · %.0f dBW · 带宽 %.0f MHz",
                               isOwn ? "本机发射干扰" : "来袭干扰压制",
                               static_cast<double>(j.frequencyGhz), static_cast<double>(j.erpDbw),
                               static_cast<double>(j.bandwidthMhz));
        }
    } else {
        ImGui::TextDisabled("当前频段无活跃电子干扰源");
    }

    ImGui::TextDisabled("ESM / RWR 截获辐射源: %zu 条", focusedEsm_.size());
    for (std::size_t i = 0; i < std::min<std::size_t>(focusedEsm_.size(), 3u); ++i) {
        const auto& e = focusedEsm_[i];
        const bool hasGeo = (e.flags & ipc::kTactEsmHasGeo) != 0u && e.crlbCep50M > 1.0f;
        if (hasGeo) {
            ImGui::Text("  #%zu  %.0f dBm · 定位 CEP50 %.0f m", i + 1, static_cast<double>(e.receivedDbm),
                        static_cast<double>(e.crlbCep50M));
        } else {
            ImGui::Text("  #%zu  %.0f dBm · 仅测向%s", i + 1, static_cast<double>(e.receivedDbm),
                        (e.flags & ipc::kTactEsmWarning) ? " [告警]" : "");
        }
    }
    ImGui::End();
}

// ============================================================================
// Plugin 4: 5-D Weapon DLZ / LCOS / CCIP & Mounted Stores
// ============================================================================
void MissionGui::drawWeaponPanel(const DockRect& rect) const {
    const float k = uiScale();
    ImGui::SetNextWindowPos(ImVec2(rect.x, rect.y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(rect.w, rect.h), ImGuiCond_Always);
    if (!beginMandatoryPanel("武器包线与挂载###weapon_panel", ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::End();
        return;
    }

    if (!focusedWeapons_.empty()) {
        const auto& w = focusedWeapons_[0];
        if ((w.flags & ipc::kTactWeaponHasDlz) != 0u && w.rMaxM > 100.0f) {
            const bool inNez = (w.flags & ipc::kTactWeaponInNez) != 0u;
            ImGui::Text("%s", w.storeName[0] ? w.storeName : "空空导弹");
            ImGui::SameLine();
            ImGui::TextColored(inNez ? tint(0.32f, 0.94f, 0.56f) : cautionText(),
                               inNez ? "[不可逃逸区 · 可发射]" : "[射程包线解算中]");

            // Visual 5-D DLZ horizontal ruler bar (Rmin | NEZ | Rmax | Raero)
            const float barW = ImGui::GetContentRegionAvail().x;
            const float barH = 22.0f * k;
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
                dl->AddLine(ImVec2(tx, p0.y - 3.0f), ImVec2(tx, p0.y + barH + 3.0f), colour(255, 255, 255, 255), 2.2f * k);
            }
            dl->AddRect(p0, ImVec2(p0.x + barW, p0.y + barH), colour(82, 112, 146, 150), 3.0f * k);

            ImGui::TextDisabled("Rmin %.1f · NEZ %.1f · Rmax %.1f · Raero %.1f km",
                                static_cast<double>(w.rMinM) * 1e-3, static_cast<double>(w.rNeM) * 1e-3,
                                static_cast<double>(w.rMaxM) * 1e-3, static_cast<double>(w.rAeroM) * 1e-3);
        }
        if ((w.flags & ipc::kTactWeaponHasLcos) != 0u) {
            ImGui::Text("机炮 LCOS 解算: 初速 %.0f m/s · 阻力系数 %.4f",
                        static_cast<double>(w.gunMuzzleSpeedMs), static_cast<double>(w.gunDragK));
        }
        if ((w.flags & ipc::kTactWeaponHasCcip) != 0u) {
            ImGui::Text("炸弹 CCIP 落点预测: 已激活（含风场补偿）");
        }
    } else if (!supportsRadar_ && hasOwnState_) {
        ImGui::TextColored(cautionText(), "飞行弹药 / 无火控载具: %s", name_.c_str());
        ImGui::Text("速度 %.0f m/s (Ma %.2f) · 过载 %.1f G", ownState_.airspeedTrueMs, ownState_.mach, ownState_.loadFactor);
        ImGui::TextDisabled("详细导引头焦平面/惯导协方差见武器诊断流");
    } else {
        ImGui::TextDisabled("当前无激活武器火控解算");
    }

    if (mirrored_ && valid_ && record_->storeCount > 0) {
        ImGui::Separator();
        ImGui::TextDisabled("外挂弹药 (%u 挂点):", static_cast<unsigned>(record_->storeCount));
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
    ImGui::End();
}

// ============================================================================
// Plugin 5: 6-DOF Aerodynamics, Flight Controls & up to 8-Engine / Rotor Panel
// ============================================================================
void MissionGui::drawAeroPanel(const DockRect& rect) const {
    const float k = uiScale();
    ImGui::SetNextWindowPos(ImVec2(rect.x, rect.y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(rect.w, rect.h), ImGuiCond_Always);
    if (!beginMandatoryPanel("气动舵面与推进###aero_panel", ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::End();
        return;
    }
    if (!hasOwnState_) {
        ImGui::TextDisabled("暂无状态数据");
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
        const float barW = std::max(40.0f * k, ImGui::GetContentRegionAvail().x - lblW - valW - 10.0f * k);
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
    ImGui::End();
}

// ============================================================================
// Plugin 6: Link 16 / C2 / Mode 5 IFF / PNT & Native Multi-Sensor Fusion
// ============================================================================
void MissionGui::drawCommsPanel(const DockRect& rect) const {
    ImGui::SetNextWindowPos(ImVec2(rect.x, rect.y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(rect.w, rect.h), ImGuiCond_Always);
    if (!beginMandatoryPanel("数据链与融合航迹###comms_panel", ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::End();
        return;
    }

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
    ImGui::Text("Link 16 活跃链路: %d · C2 授权: %d · IFF: %d", link16Count, c2Count, iffCount);
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
    ImGui::End();
}

} // namespace fsim::ui
