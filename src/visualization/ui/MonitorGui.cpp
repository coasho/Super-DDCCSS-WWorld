#include "visualization/ui/MonitorGui.h"
#include "visualization/ui/MandatoryPanel.h"

#include "foundation/core/Units.h"
#include "visualization/scene/CameraController.h"
#include "visualization/ui/Colour.h"

#include <vsgImGui/imgui.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>

namespace fsim::ui {

namespace {

struct Layout {
    float scale;
    float margin;
    ImVec2 display;
    float topBarMaxWidth;
    float listWidth, listHeight, listTop;
};

Layout layout() {
    Layout l;
    l.scale = uiScale();
    l.margin = 10.0f * l.scale;
    l.display = ImGui::GetIO().DisplaySize;
    // Scheme 3 Top Workspace Bar stays strictly inside [margin .. 78% width] on wide 4K screens
    // (keeping top-right corner clear of desktop clocks/overlays) and cleanly fits 1600x900 @ 150% DPI.
    l.topBarMaxWidth = std::min(l.display.x - 2.0f * l.margin, std::max(720.0f * l.scale, l.display.x * 0.78f));
    l.listWidth = std::min(530.0f * l.scale, std::max(280.0f, l.display.x * 0.45f));
    l.listHeight = std::min(215.0f * l.scale, std::max(100.0f, l.display.y * 0.25f));
    l.listTop = std::max(l.margin, l.display.y - l.listHeight - l.margin);
    return l;
}

std::uint32_t constraintsOf(const ipc::FlightStatus& status) noexcept {
    const double c = status.derived[ipc::kDerivedConstraints];
    return c > 0.0 && c < 65536.0 ? static_cast<std::uint32_t>(c) : 0u;
}

} // namespace

MonitorGui::MonitorGui(std::shared_ptr<ViewerControls> controls, std::string aircraft)
    : controls_(std::move(controls)), aircraft_(std::move(aircraft)) {}

int MonitorGui::severity(const ipc::FlightStatus& status, const char** word) noexcept {
    const std::uint32_t c = constraintsOf(status);
    const char* say = "";
    int level = 0;
    if (c & ipc::kConstraintUpset) say = "失控", level = 2;
    else if (c & ipc::kConstraintDeviating) say = "偏航", level = 2;
    else if (c & ipc::kConstraintLowEnergy) say = "低能量", level = 1;
    if (word) *word = say;
    return level;
}

void MonitorGui::record(vsg::CommandBuffer&) const {
    drawLabels();
    drawMonitor();
    drawVehicleList();
}

void MonitorGui::drawLabels() const {
    // Solid tactical entity badges with geometric IFF markers (clearly distinct from quiet borderless 3-D zone text!).
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const float k = uiScale();
    for (const Label& l : labels_) {
        const ImVec2 size = ImGui::CalcTextSize(l.text.c_str());
        const float iconSpace = 14.0f * k;
        const float padX = 6.0f * k;
        const float padY = 2.5f * k;
        const float boxW = iconSpace + size.x + padX * 2.0f;
        const float boxH = size.y + padY * 2.0f;
        const ImVec2 lo(l.x + 10.0f * k, l.y - boxH * 0.5f);
        const ImVec2 hi(lo.x + boxW, lo.y + boxH);

        const ImU32 bg = l.severity >= 2 ? colour(196, 40, 36, 240)
                       : l.severity == 1 ? colour(228, 166, 28, 240)
                       : l.selected      ? colour(208, 118, 18, 238)
                       : l.iconKind == 2 ? colour(112, 24, 30, 224)
                       : l.iconKind == 1 ? colour(16, 36, 62, 224)
                                         : colour(22, 29, 42, 222);
        const ImU32 border = l.selected      ? colour(255, 232, 162, 245)
                           : l.iconKind == 2 ? colour(248, 108, 104, 220)
                           : l.iconKind == 1 ? colour(74, 188, 255, 220)
                           : l.iconKind == 3 ? colour(250, 198, 72, 220)
                                             : colour(128, 152, 180, 165);
        const ImU32 fg = (l.severity == 1) ? colour(18, 14, 0, 255) : colour(246, 250, 255, 255);

        dl->AddRectFilled(lo, hi, bg, 3.5f * k);
        dl->AddRect(lo, hi, border, 3.5f * k, 0, (l.selected ? 1.5f : 1.0f) * k);
        dl->AddLine(ImVec2(l.x, l.y), ImVec2(lo.x, l.y), border, 1.3f * k);

        // Geometric IFF glyph on left of badge:
        // 1 = Blue solid diamond, 2 = Red hollow diamond, 3 = Missile/weapon triangle, 0 = neutral dot
        const float ix = lo.x + padX + 4.5f * k;
        const float iy = l.y;
        const float ir = 4.2f * k;
        if (l.iconKind == 1) {
            const ImVec2 pts[4] = {{ix, iy - ir}, {ix + ir, iy}, {ix, iy + ir}, {ix - ir, iy}};
            dl->AddConvexPolyFilled(pts, 4, fg);
        } else if (l.iconKind == 2) {
            const ImVec2 pts[4] = {{ix, iy - ir}, {ix + ir, iy}, {ix, iy + ir}, {ix - ir, iy}};
            dl->AddPolyline(pts, 4, fg, ImDrawFlags_Closed, 1.4f * k);
        } else if (l.iconKind == 3) {
            dl->AddTriangleFilled(ImVec2(ix, iy - ir), ImVec2(ix + ir, iy + ir * 0.85f), ImVec2(ix - ir, iy + ir * 0.85f), fg);
        } else {
            dl->AddCircleFilled(ImVec2(ix, iy), ir * 0.65f, fg);
        }
        dl->AddText(ImVec2(lo.x + padX + iconSpace, lo.y + padY), fg, l.text.c_str());
    }
}

void MonitorGui::drawMonitor() const {
    const Layout l = layout();
    ImGui::SetNextWindowPos(ImVec2(l.margin, l.margin), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(std::min(520.0f * l.scale, l.topBarMaxWidth), 0.0f),
                                        ImVec2(l.topBarMaxWidth, std::max(140.0f, l.listTop - 2.0f * l.margin)));
    if (!beginMandatoryPanel("飞行监控###flightsim", ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar)) {
        ImGui::End();
        return;
    }

    const double fps = controls_->fps.load(std::memory_order_relaxed);
    const double frameMs = controls_->frameMs.load(std::memory_order_relaxed);
    const double sps = controls_->simThroughput.load(std::memory_order_relaxed);
    const double simTime = controls_->simTime.load(std::memory_order_relaxed);
    const bool compact = (l.display.x / l.scale) < 1200.0f;

    std::size_t live = 0;
    if (!vehicles_.empty()) {
        for (const auto& v : vehicles_) live += v.alive ? 1 : 0;
    } else if (batch_) {
        live = batch_->states.size();
    }

    // --- Row 1: Prominent FPS | Role-Based Workspace Preset Tabs | Custom Panel Popover | Camera ---
    const ImVec4 fpsCol = fps >= 55.0 ? tint(0.32f, 0.94f, 0.56f)
                        : fps >= 30.0 ? cautionText()
                                      : warningText();
    ImGui::TextColored(fpsCol, "帧率 %.0f FPS", fps);
    ImGui::SameLine();
    ImGui::TextDisabled("(%.1f ms) · %zu 架", frameMs, live);

    // Role-based Workspace Preset Switcher (Scheme 3)
    int activePreset = std::clamp(controls_->workspacePreset.load(std::memory_order_relaxed), 0, kWorkspacePresetCount - 1);
    const struct PresetDef {
        WorkspacePreset id;
        const char* label;
    } presets[] = {
        {WorkspacePreset::Clean,    "观战纯净"},
        {WorkspacePreset::AirRadar, "空战雷达"},
        {WorkspacePreset::Emso,     "电子战"},
        {WorkspacePreset::Weapon,   "武器导引"},
        {WorkspacePreset::Flight,   "飞控气动"},
    };
    ImGui::SameLine(0.0f, 14.0f * l.scale);
    for (const auto& p : presets) {
        const bool isAct = (activePreset == static_cast<int>(p.id));
        if (isAct) {
            ImGui::PushStyleColor(ImGuiCol_Button, tint(0.14f, 0.44f, 0.72f, 0.96f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, tint(0.18f, 0.52f, 0.82f, 1.0f));
        }
        if (ImGui::SmallButton(p.label)) {
            activePreset = static_cast<int>(p.id);
            controls_->workspacePreset.store(activePreset, std::memory_order_relaxed);
            controls_->customPanelMask.store(panelsForWorkspace(p.id, controls_->customPanelMask.load(std::memory_order_relaxed)),
                                             std::memory_order_relaxed);
        }
        if (isAct) ImGui::PopStyleColor(2);
        ImGui::SameLine(0.0f, 5.0f * l.scale);
    }

    // "+ 面板" extensible plugin drawer popover (lets user toggle any registered panel -> Custom preset)
    const bool isCustom = (activePreset == static_cast<int>(WorkspacePreset::Custom));
    if (isCustom) ImGui::PushStyleColor(ImGuiCol_Button, tint(0.14f, 0.44f, 0.72f, 0.96f));
    if (ImGui::SmallButton("+ 面板")) ImGui::OpenPopup("workspace_plugins_popup");
    if (isCustom) ImGui::PopStyleColor();

    if (ImGui::BeginPopup("workspace_plugins_popup")) {
        ImGui::TextDisabled("自定义停靠面板（按当前机型自动适配显隐）");
        ImGui::Separator();
        std::uint32_t curMask = panelsForWorkspace(static_cast<WorkspacePreset>(activePreset),
                                                   controls_->customPanelMask.load(std::memory_order_relaxed));
        bool changed = false;
        auto modCheck = [&](const char* name, std::uint32_t bit, bool mandatory = false) {
            bool on = (curMask & bit) != 0u;
            if (mandatory) ImGui::BeginDisabled();
            if (ImGui::Checkbox(name, &on)) {
                if (on) curMask |= bit;
                else curMask &= ~bit;
                changed = true;
            }
            if (mandatory) ImGui::EndDisabled();
        };
        modCheck("机载雷达 B-Scope / PPI（雷达机型必显）", kPanelRadar, true);
        modCheck("雷达底层可视化（距离-多普勒 / 烧穿 / 方向图）", kPanelSignal);
        modCheck("电子战频谱与侦察（EA / ESM / CRLB）", kPanelEw);
        modCheck("武器包线与导引（DLZ / LCOS / CCIP）", kPanelWeapon);
        modCheck("6-DOF 气动舵面与推进（含 8 发/旋翼）", kPanelAero);
        modCheck("数据链 / IFF / 融合航迹", kPanelComms);
        modCheck("左下编队列表", kPanelFleet);
        if (changed) {
            controls_->customPanelMask.store(curMask, std::memory_order_relaxed);
            controls_->workspacePreset.store(static_cast<int>(WorkspacePreset::Custom), std::memory_order_relaxed);
        }
        ImGui::EndPopup();
    }

    // Compact Camera Mode Combo + Reset right on Row 1 (or wrapped cleanly on compact screens)
    if (!compact) ImGui::SameLine(0.0f, 14.0f * l.scale);
    int camMode = std::clamp(controls_->cameraMode.load(std::memory_order_relaxed), 0, world::CameraController::kModeCount - 1);
    const char* camModes[] = {"追踪视角", "环绕视角", "俯视总览", "自由视角"};
    ImGui::SetNextItemWidth(102.0f * l.scale);
    if (ImGui::Combo("##camera", &camMode, camModes, world::CameraController::kModeCount))
        controls_->cameraMode.store(camMode, std::memory_order_relaxed);
    ImGui::SameLine(0.0f, 4.0f * l.scale);
    if (ImGui::SmallButton("重置[R]")) controls_->cameraReset.store(true, std::memory_order_relaxed);

    // --- Row 2: Clock / Playback | Focused Target Telemetry | Quick 3-D Overlays Popover ---
    if (source_.mirror) {
        if (!source_.attached) {
            ImGui::TextColored(cautionText(), "等待仿真接入...");
            if (!source_.available.empty()) {
                ImGui::SameLine();
                if (ImGui::SmallButton("接入世界")) ImGui::OpenPopup("mirror_world_popup");
            }
        } else {
            ImGui::Text("世界: %s", source_.world.c_str());
            ImGui::SameLine();
            ImGui::TextDisabled("t=%.1fs · %.0f 步/s", source_.simTime, source_.vehicleStepsPerSecond);
            if (!source_.publisherAlive) { ImGui::SameLine(); ImGui::TextColored(warningText(), "已退出"); }
            else if (source_.ageSeconds > 2.0) { ImGui::SameLine(); ImGui::TextColored(cautionText(), "停更%.0fs", source_.ageSeconds); }
            if (source_.available.size() > 1) {
                ImGui::SameLine();
                if (ImGui::SmallButton("切换")) ImGui::OpenPopup("mirror_world_popup");
            }
        }
        if (ImGui::BeginPopup("mirror_world_popup")) {
            for (std::size_t i = 0; i < source_.available.size(); ++i) {
                const bool sel = (source_.available[i] == source_.world);
                if (ImGui::Selectable(source_.available[i].c_str(), sel) && !sel)
                    controls_->attachWorld.store(static_cast<int>(i), std::memory_order_relaxed);
            }
            ImGui::EndPopup();
        }
    } else {
        bool paused = controls_->paused.load(std::memory_order_relaxed);
        if (ImGui::SmallButton(paused ? "继续[Space]" : "暂停[Space]"))
            controls_->paused.store(!paused, std::memory_order_relaxed);
        ImGui::SameLine(0.0f, 4.0f * l.scale);
        if (ImGui::SmallButton("单步[.]")) controls_->singleStep.store(true, std::memory_order_relaxed);
        ImGui::SameLine(0.0f, 6.0f * l.scale);
        const double start = controls_->replayStart.load(std::memory_order_relaxed);
        const double end = controls_->replayEnd.load(std::memory_order_relaxed);
        if (end > start) {
            float at = static_cast<float>(simTime);
            ImGui::SetNextItemWidth(145.0f * l.scale);
            if (ImGui::SliderFloat("##timeline", &at, static_cast<float>(start), static_cast<float>(end), "%.1f s"))
                controls_->seekTo.store(static_cast<double>(at), std::memory_order_relaxed);
        } else {
            ImGui::TextDisabled("t=%.1fs · %.0f 步/s", simTime, sps);
        }
        if (!compact && !aircraft_.empty()) {
            ImGui::SameLine(0.0f, 8.0f * l.scale);
            ImGui::TextDisabled("%s", aircraft_.c_str());
        }
    }

    // Focused target compact telemetry inline
    if (batch_ && !batch_->states.empty()) {
        const int sel = std::clamp(controls_->selectedVehicle.load(std::memory_order_relaxed), 0,
                                   static_cast<int>(batch_->states.size()) - 1);
        const auto& s = batch_->states[static_cast<std::size_t>(sel)];
        const auto* meta = static_cast<std::size_t>(sel) < vehicles_.size() ? &vehicles_[static_cast<std::size_t>(sel)] : nullptr;
        ImGui::SameLine(0.0f, 12.0f * l.scale);
        ImGui::TextColored(tint(0.45f, 0.86f, 1.0f), "[Tab] %s", (meta && meta->alive) ? meta->name.c_str() : "载具");
        if (!compact) {
            ImGui::SameLine(0.0f, 6.0f * l.scale);
            ImGui::TextDisabled("海高 %.0fm · 空速 %.0fm/s · 航向 %.0f°",
                                s.altitudeMslM, s.airspeedTrueMs, units::radiansToDegrees(s.eulerRad[2]));
        }
    }

    // Quick 3-D Tactical Overlays Popover (always 1 click away, zero permanent screen bloat!)
    std::uint32_t mask = controls_->tacticalOverlayMask.load(std::memory_order_relaxed);
    ImGui::SameLine(0.0f, 12.0f * l.scale);
    if (mask != 0u) ImGui::PushStyleColor(ImGuiCol_Button, tint(0.14f, 0.42f, 0.56f, 0.92f));
    if (ImGui::SmallButton(mask == kOverlayAll ? "3-D 图层: 全开" : mask ? "3-D 图层: 部分" : "3-D 图层: 关"))
        ImGui::OpenPopup("tact_overlay_popup");
    if (mask != 0u) ImGui::PopStyleColor();

    if (ImGui::BeginPopup("tact_overlay_popup")) {
        ImGui::TextDisabled("3-D 态势图层（仅显示聚焦载具）");
        if (ImGui::SmallButton("全开")) mask = kOverlayAll;
        ImGui::SameLine();
        if (ImGui::SmallButton("全关 (0 ms)")) mask = 0u;
        ImGui::Separator();
        auto toggleBit = [&](const char* label, std::uint32_t bit, ImU32 swatchCol) {
            bool on = (mask & bit) != 0u;
            const ImVec2 p = ImGui::GetCursorScreenPos();
            const float dotR = 3.5f * l.scale;
            const float cy = p.y + ImGui::GetFrameHeight() * 0.5f;
            ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(p.x + dotR, cy), dotR, swatchCol);
            ImGui::Dummy(ImVec2(dotR * 2.4f, 0.0f));
            ImGui::SameLine(0.0f, 4.0f * l.scale);
            if (ImGui::Checkbox(label, &on)) {
                if (on) mask |= bit;
                else mask &= ~bit;
            }
        };
        toggleBit("雷达立体走廊与烧穿", kOverlayRadar, colour(56, 184, 255, 255));
        toggleBit("电子干扰锥 (EA)", kOverlayJammer, colour(198, 96, 252, 255));
        toggleBit("电子侦察与误差椭球", kOverlayEsm, colour(252, 206, 56, 255));
        toggleBit("武器射程门 (DLZ/LCOS)", kOverlayWeapons, colour(50, 224, 132, 255));
        toggleBit("数据链 / IFF / 导航", kOverlayLinks, colour(46, 228, 168, 255));
        toggleBit("SAR / 气象条带", kOverlaySwaths, colour(56, 208, 236, 255));
        controls_->tacticalOverlayMask.store(mask, std::memory_order_relaxed);
        ImGui::EndPopup();
    }

    // Publish exact live top-bar extents for right dock & 3-D zone label avoidance!
    const ImVec2 winPos = ImGui::GetWindowPos();
    const ImVec2 winSz = ImGui::GetWindowSize();
    controls_->topBarBottomPx.store(winPos.y + winSz.y, std::memory_order_relaxed);
    controls_->topBarRightPx.store(winPos.x + winSz.x, std::memory_order_relaxed);
    ImGui::End();
}

void MonitorGui::drawStatus(const sim::VehicleState& s, const ipc::FlightStatus& status, bool state) const {
    const double* d = status.derived;
    if (!(d[ipc::kDerivedMassKg] > 0.0)) return;
    const float gap = 12.0f * uiScale();
    char text[64];
    bool first = true;
    enum class Tone { Plain, Dim, Caution, Warning };
    const auto item = [&](Tone tone) {
        if (!first) {
            ImGui::SameLine(0.0f, gap);
            if (ImGui::GetContentRegionAvail().x < ImGui::CalcTextSize(text).x) ImGui::NewLine();
        }
        first = false;
        switch (tone) {
        case Tone::Plain: ImGui::TextUnformatted(text); break;
        case Tone::Dim: ImGui::TextDisabled("%s", text); break;
        case Tone::Caution: ImGui::TextColored(cautionText(), "%s", text); break;
        case Tone::Warning: ImGui::TextColored(warningText(), "%s", text); break;
        }
        return ImGui::IsItemHovered();
    };
    const bool flying = !s.onGround && !s.crashed;
    if (state && flying) {
        const char* word = "";
        if (const int level = severity(status, &word)) {
            std::snprintf(text, sizeof text, "%s", word);
            item(level >= 2 ? Tone::Warning : Tone::Caution);
        }
    }
    const double mass = d[ipc::kDerivedMassKg];
    std::snprintf(text, sizeof text, mass < 100.0 ? "质量 %.2f kg" : "质量 %.0f kg", mass);
    if (item(Tone::Plain)) {
        if (std::isfinite(s.fuelKg) && s.fuelKg > 0.0) ImGui::SetTooltip("当前总质量（含燃油 %.0f kg）", s.fuelKg);
        else ImGui::SetTooltip("当前总质量");
    }
    const double stall = d[ipc::kDerivedStallCasMs], least = d[ipc::kDerivedLeastCasMs], floor = d[ipc::kDerivedFloorCasMs];
    if (!(stall > 0.0) || !(least > 0.0)) return;
    const double cas = s.airspeedCalibratedMs;
    std::snprintf(text, sizeof text, "表速 %.0f", cas);
    if (item(flying && cas < stall ? Tone::Warning : flying && cas < least ? Tone::Caution : Tone::Plain))
        ImGui::SetTooltip("当前校准空速 (m/s)");
    std::snprintf(text, sizeof text, "最低 %.0f", least);
    if (item(Tone::Dim)) ImGui::SetTooltip("指令允许的最低校准空速 (m/s)");
    std::snprintf(text, sizeof text, "失速 %.0f", stall);
    if (item(Tone::Dim))
        ImGui::SetTooltip("当前重量下的失速速度 (m/s，载荷倍率 %.3f)",
                          d[ipc::kDerivedLoadRatio] > 0.0 ? d[ipc::kDerivedLoadRatio] : 1.0);
    if (flying && floor > 0.0) {
        std::snprintf(text, sizeof text, "底速 %.0f", floor);
        if (item(Tone::Caution)) ImGui::SetTooltip("保持高度的最低保护速度 (m/s)");
    }
    const double cap = d[ipc::kDerivedBankCapRad];
    if (flying && cap > 0.0 && cap < 1.05) {
        const double bank = std::abs(units::radiansToDegrees(s.eulerRad[0])), most = units::radiansToDegrees(cap);
        std::snprintf(text, sizeof text, "坡度 ≤ %.0f°", most);
        if (item(bank > most + 5.0 ? Tone::Caution : Tone::Dim))
            ImGui::SetTooltip("当前空速允许的最大转弯坡度");
    }
}

void MonitorGui::drawVehicleList() const {
    const auto preset = static_cast<WorkspacePreset>(
        std::clamp(controls_->workspacePreset.load(std::memory_order_relaxed), 0, kWorkspacePresetCount - 1));
    const std::uint32_t activePanels = panelsForWorkspace(preset, controls_->customPanelMask.load(std::memory_order_relaxed));
    if (!batch_ || (activePanels & kPanelFleet) == 0u) {
        controls_->leftDockRightPx.store(0.0f, std::memory_order_relaxed);
        controls_->leftDockTopPx.store(0.0f, std::memory_order_relaxed);
        return;
    }

    const Layout l = layout();
    ImGui::SetNextWindowPos(ImVec2(l.margin, l.listTop), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(l.listWidth, l.listHeight), ImGuiCond_FirstUseEver);
    if (!beginMandatoryPanel("载具列表###vehicles")) {
        ImGui::End();
        return;
    }
    const int selected = controls_->selectedVehicle.load(std::memory_order_relaxed);
    if (status_ && selected >= 0 && static_cast<std::size_t>(selected) < status_->size() &&
        static_cast<std::size_t>(selected) < batch_->states.size() &&
        (static_cast<std::size_t>(selected) >= vehicles_.size() || vehicles_[static_cast<std::size_t>(selected)].alive))
        drawStatus(batch_->states[static_cast<std::size_t>(selected)], (*status_)[static_cast<std::size_t>(selected)], false);

    const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY |
                                  ImGuiTableFlags_SizingStretchProp;
    if (ImGui::BeginTable("vehicles", 7, flags)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("名称", ImGuiTableColumnFlags_WidthStretch, 2.35f);
        ImGui::TableSetupColumn("控制", ImGuiTableColumnFlags_WidthStretch, 1.05f);
        ImGui::TableSetupColumn("海高 m", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("离地 m", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("空速", ImGuiTableColumnFlags_WidthStretch, 0.9f);
        ImGui::TableSetupColumn("航向", ImGuiTableColumnFlags_WidthStretch, 0.8f);
        ImGui::TableSetupColumn("状态", ImGuiTableColumnFlags_WidthStretch, 1.1f);
        ImGui::TableHeadersRow();
        for (std::size_t i = 0; i < batch_->states.size(); ++i) {
            const auto* meta = i < vehicles_.size() ? &vehicles_[i] : nullptr;
            if (meta && !meta->alive) continue;
            const auto& s = batch_->states[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            char label[96];
            if (meta) std::snprintf(label, sizeof label, "%s##%zu", meta->name.c_str(), i);
            else std::snprintf(label, sizeof label, "载具 %zu##%zu", i, i);
            if (ImGui::Selectable(label, static_cast<int>(i) == selected, ImGuiSelectableFlags_SpanAllColumns))
                controls_->selectedVehicle.store(static_cast<int>(i), std::memory_order_relaxed);
            ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(meta ? meta->level : "");
            if (s.diverged) {
                for (int c = 2; c <= 5; ++c) { ImGui::TableSetColumnIndex(c); ImGui::TextUnformatted("-"); }
                ImGui::TableSetColumnIndex(6);
                ImGui::TextColored(warningText(), "发散");
                continue;
            }
            ImGui::TableSetColumnIndex(2); ImGui::Text("%.0f", s.altitudeMslM);
            ImGui::TableSetColumnIndex(3); ImGui::Text("%.0f", s.altitudeAglM);
            ImGui::TableSetColumnIndex(4); ImGui::Text("%.0f", s.airspeedTrueMs);
            ImGui::TableSetColumnIndex(5); ImGui::Text("%.0f°", units::radiansToDegrees(s.eulerRad[2]));
            ImGui::TableSetColumnIndex(6);
            const char* word = "";
            const int level = status_ && i < status_->size() && !s.onGround ? severity((*status_)[i], &word) : 0;
            if (level > 0) ImGui::TextColored(level >= 2 ? warningText() : cautionText(), "%s", word);
            else if (s.crashed) ImGui::TextColored(warningText(), "坠毁");
            else ImGui::TextUnformatted(s.onGround ? "地面" : "空中");
        }
        ImGui::EndTable();
    }
    const ImVec2 pos = ImGui::GetWindowPos();
    const ImVec2 sz = ImGui::GetWindowSize();
    controls_->leftDockRightPx.store(pos.x + sz.x, std::memory_order_relaxed);
    controls_->leftDockTopPx.store(pos.y, std::memory_order_relaxed);
    ImGui::End();
}

} // namespace fsim::ui
