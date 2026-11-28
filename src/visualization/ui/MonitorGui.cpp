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
    float panelScale;
    float margin;
    ImVec2 display;
    float topBarMaxWidth;
    float listWidth, listHeight, listTop;
};

Layout layout(const ViewerControls* controls) {
    Layout l;
    l.scale = uiScale();
    l.panelScale = controls ? std::clamp(controls->panelScale.load(std::memory_order_relaxed), 0.75f, 1.35f) : 1.0f;
    l.margin = 10.0f * l.scale;
    l.display = ImGui::GetIO().DisplaySize;
    // Frameless top command strip stays cleanly inside [margin .. 76% width] on 4K (clearing top-right clock)
    // and wraps cleanly without clipping on 1600x900 @ 150% DPI (B-43).
    l.topBarMaxWidth = std::min(l.display.x - 2.0f * l.margin, std::max(720.0f * l.scale, l.display.x * 0.76f));
    const float ps = l.panelScale;
    l.listWidth = std::min(510.0f * l.scale * ps, std::max(280.0f, l.display.x * 0.42f));
    l.listHeight = std::min((195.0f * ps + 28.0f) * l.scale, std::max(100.0f, l.display.y * 0.28f));
    l.listTop = std::max(l.margin, l.display.y - l.listHeight - l.margin);
    return l;
}

std::uint32_t constraintsOf(const ipc::FlightStatus& status) noexcept {
    const double c = status.derived[ipc::kDerivedConstraints];
    return c > 0.0 && c < 65536.0 ? static_cast<std::uint32_t>(c) : 0u;
}

void drawVerticalHairline(float s) {
    ImGui::SameLine(0.0f, 7.0f * s);
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float fh = ImGui::GetFrameHeight();
    ImGui::GetWindowDrawList()->AddLine(ImVec2(p.x, p.y + fh * 0.16f), ImVec2(p.x, p.y + fh * 0.84f),
                                        colour(148, 178, 214, 58), 1.0f);
    ImGui::Dummy(ImVec2(1.0f, fh));
    ImGui::SameLine(0.0f, 7.0f * s);
}

void drawIffBadgeGlyph(ImDrawList* dl, float ix, float iy, float ir, std::uint8_t iconKind, ImU32 fg, float scale) {
    if (iconKind == 1) {
        const ImVec2 pts[4] = {{ix, iy - ir}, {ix + ir, iy}, {ix, iy + ir}, {ix - ir, iy}};
        dl->AddConvexPolyFilled(pts, 4, fg);
    } else if (iconKind == 2) {
        const ImVec2 pts[4] = {{ix, iy - ir}, {ix + ir, iy}, {ix, iy + ir}, {ix - ir, iy}};
        dl->AddPolyline(pts, 4, fg, ImDrawFlags_Closed, 1.4f * scale);
    } else if (iconKind == 3) {
        dl->AddTriangleFilled(ImVec2(ix, iy - ir), ImVec2(ix + ir, iy + ir * 0.85f), ImVec2(ix - ir, iy + ir * 0.85f), fg);
    } else {
        dl->AddCircleFilled(ImVec2(ix, iy), ir * 0.65f, fg);
    }
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
    drawReplayBar();
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

        drawIffBadgeGlyph(dl, lo.x + padX + 4.5f * k, l.y, 4.2f * k, l.iconKind, fg, k);
        dl->AddText(ImVec2(lo.x + padX + iconSpace, lo.y + padY), fg, l.text.c_str());
    }
}

void MonitorGui::drawMonitor() const {
    const Layout l = layout(controls_.get());
    const float s = l.scale;
    const bool compact = (l.display.x / s) < 1380.0f;

    // Style A: Sleek Frameless Single-Row Viewport Command Strip (no bulky window title bar!).
    ImGui::SetNextWindowPos(ImVec2(l.margin, l.margin), ImGuiCond_Always);
    ImGui::SetNextWindowSizeConstraints(ImVec2(0.0f, 0.0f),
                                        ImVec2(l.topBarMaxWidth, std::max(120.0f, l.listTop - 2.0f * l.margin)));

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 9.0f * s);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f * s, 5.5f * s));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 5.5f * s);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f * s, 4.0f * s));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(5.0f * s, 5.0f * s));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, tint(0.043f, 0.063f, 0.094f, 0.88f));
    ImGui::PushStyleColor(ImGuiCol_Border, tint(0.46f, 0.62f, 0.82f, 0.28f));

    const ImGuiWindowFlags barFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize;
    if (!beginMandatoryPanel("飞行监控###flightsim", barFlags)) {
        ImGui::End();
        ImGui::PopStyleColor(2);
        ImGui::PopStyleVar(5);
        return;
    }

    const double fps = controls_->fps.load(std::memory_order_relaxed);
    const double frameMs = controls_->frameMs.load(std::memory_order_relaxed);
    const double sps = controls_->simThroughput.load(std::memory_order_relaxed);
    const double simTime = controls_->simTime.load(std::memory_order_relaxed);
    std::size_t live = 0;
    if (!vehicles_.empty()) {
        for (const auto& v : vehicles_) live += v.alive ? 1 : 0;
    } else if (batch_) {
        live = batch_->states.size();
    }

    // =========================================================================
    // CLUSTER 1: Live FPS Indicator | Focus Vehicle Selector Pill | Telemetry
    // =========================================================================
    {
        const ImVec2 p = ImGui::GetCursorScreenPos();
        const float fh = ImGui::GetFrameHeight();
        const float dotR = 3.6f * s;
        const ImU32 dotCol = fps >= 55.0 ? colour(59, 224, 131, 255)
                           : fps >= 30.0 ? colour(245, 182, 56, 255)
                                         : colour(242, 87, 87, 255);
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(p.x + dotR, p.y + fh * 0.5f), dotR, dotCol);
        ImGui::Dummy(ImVec2(dotR * 2.4f, fh));
        ImGui::SameLine(0.0f, 3.0f * s);
        ImGui::AlignTextToFramePadding();
        const ImVec4 fpsCol = fps >= 55.0 ? tint(0.32f, 0.94f, 0.56f)
                            : fps >= 30.0 ? cautionText()
                                          : warningText();
        ImGui::TextColored(fpsCol, "%.0f FPS", fps);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("帧耗时 %.1f ms · %zu 架在空 · %.0f 步/s", frameMs, live, sps);
        }
    }

    // Focus target quick-switch pill ("◆ viper-1 ▾")
    if (batch_ && !batch_->states.empty()) {
        const int nVeh = static_cast<int>(batch_->states.size());
        const int sel = std::clamp(controls_->selectedVehicle.load(std::memory_order_relaxed), 0, nVeh - 1);
        const auto& st = batch_->states[static_cast<std::size_t>(sel)];
        const auto* meta = static_cast<std::size_t>(sel) < vehicles_.size() ? &vehicles_[static_cast<std::size_t>(sel)] : nullptr;
        const char* curName = (meta && !meta->name.empty()) ? meta->name.c_str() : "载具";

        drawVerticalHairline(s);
        char focusBtn[96];
        std::snprintf(focusBtn, sizeof focusBtn, "◆ %s ▼###focus_btn", curName);
        ImGui::PushStyleColor(ImGuiCol_Button, tint(0.42f, 0.28f, 0.08f, 0.72f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, tint(0.56f, 0.36f, 0.10f, 0.88f));
        if (ImGui::Button(focusBtn)) ImGui::OpenPopup("focus_vehicle_popup");
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("点击直接切换聚焦载具（或按 Tab 轮切）");

        if (ImGui::BeginPopup("focus_vehicle_popup")) {
            ImGui::TextDisabled("聚焦载具（%zu 架在空）", live);
            ImGui::Separator();
            for (int i = 0; i < nVeh; ++i) {
                const bool hasMeta = static_cast<std::size_t>(i) < vehicles_.size();
                if (hasMeta && !vehicles_[static_cast<std::size_t>(i)].alive) continue;
                char rowLabel[128];
                const char* nm = (hasMeta && !vehicles_[static_cast<std::size_t>(i)].name.empty())
                                     ? vehicles_[static_cast<std::size_t>(i)].name.c_str()
                                     : "载具";
                const char* tp = (hasMeta && !vehicles_[static_cast<std::size_t>(i)].type.empty())
                                     ? vehicles_[static_cast<std::size_t>(i)].type.c_str()
                                     : "";
                std::snprintf(rowLabel, sizeof rowLabel, "%s  [%s]###foc_%d", nm, tp, i);
                if (ImGui::Selectable(rowLabel, i == sel)) {
                    controls_->selectedVehicle.store(i, std::memory_order_relaxed);
                }
            }
            ImGui::EndPopup();
        }

        if (!compact) {
            ImGui::SameLine(0.0f, 6.0f * s);
            ImGui::AlignTextToFramePadding();
            ImGui::TextDisabled("%.0fm · %.0fm/s · %03.0f°",
                                st.altitudeMslM, st.airspeedTrueMs,
                                std::fmod(units::radiansToDegrees(st.eulerRad[2]) + 360.0, 360.0));
        }
    }

    // Mirror status pill (only when mirroring a live simulation world)
    if (source_.mirror) {
        drawVerticalHairline(s);
        if (!source_.attached) {
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(cautionText(), "等待仿真");
            if (!source_.available.empty()) {
                ImGui::SameLine();
                if (ImGui::Button("接入")) ImGui::OpenPopup("mirror_world_popup");
            }
        } else {
            char wBtn[96];
            std::snprintf(wBtn, sizeof wBtn, "%s %.0fs%s###world_btn", source_.world.c_str(), source_.simTime,
                          source_.available.size() > 1 ? " ▼" : "");
            if (ImGui::Button(wBtn) && source_.available.size() > 1) ImGui::OpenPopup("mirror_world_popup");
            if (!source_.publisherAlive) {
                ImGui::SameLine();
                ImGui::TextColored(warningText(), "已退出");
            } else if (source_.ageSeconds > 2.0) {
                ImGui::SameLine();
                ImGui::TextColored(cautionText(), "停更");
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
    }

    // =========================================================================
    // CLUSTER 2: Role-Based Workspace Segmented Control + Extensible "+ 面板"
    // =========================================================================
    drawVerticalHairline(s);
    int activePreset = std::clamp(controls_->workspacePreset.load(std::memory_order_relaxed), 0, kWorkspacePresetCount - 1);
    const struct PresetDef {
        WorkspacePreset id;
        const char* label;
        const char* tip;
    } presets[] = {
        {WorkspacePreset::Clean,    "纯净", "纯观战模式（关闭全部侧边与底栏面板，零遮挡）"},
        {WorkspacePreset::AirRadar, "空战", "机载雷达 + 原生信号级多普勒（可用时）+ 载具列表"},
        {WorkspacePreset::Flight,   "飞控", "本机 6-DOF 气动推进 + 数据链/融合航迹 + 载具列表"},
    };
    for (int i = 0; i < 3; ++i) {
        if (i > 0) ImGui::SameLine(0.0f, 3.0f * s);
        const bool isAct = (activePreset == static_cast<int>(presets[i].id));
        if (isAct) {
            ImGui::PushStyleColor(ImGuiCol_Button, tint(0.14f, 0.46f, 0.78f, 0.96f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, tint(0.18f, 0.54f, 0.88f, 1.0f));
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, tint(0.10f, 0.14f, 0.20f, 0.65f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, tint(0.16f, 0.22f, 0.32f, 0.85f));
        }
        if (ImGui::Button(presets[i].label)) {
            activePreset = static_cast<int>(presets[i].id);
            controls_->workspacePreset.store(activePreset, std::memory_order_relaxed);
            controls_->customPanelMask.store(panelsForWorkspace(presets[i].id, 0u), std::memory_order_relaxed);
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", presets[i].tip);
    }

    ImGui::SameLine(0.0f, 4.0f * s);
    const bool isCustomOrExt = (activePreset == static_cast<int>(WorkspacePreset::Custom) ||
                                activePreset == static_cast<int>(WorkspacePreset::Emso) ||
                                activePreset == static_cast<int>(WorkspacePreset::Weapon));
    if (isCustomOrExt) ImGui::PushStyleColor(ImGuiCol_Button, tint(0.14f, 0.46f, 0.78f, 0.92f));
    if (ImGui::Button("+ 面板 ▼")) ImGui::OpenPopup("workspace_plugins_popup");
    if (isCustomOrExt) ImGui::PopStyleColor();

    if (ImGui::BeginPopup("workspace_plugins_popup")) {
        ImGui::TextDisabled("自由勾选面板（100%% 真实原生数据，不再强制显示雷达）");
        ImGui::Separator();
        std::uint32_t curMask = panelsForWorkspace(static_cast<WorkspacePreset>(activePreset),
                                                   controls_->customPanelMask.load(std::memory_order_relaxed));
        bool changed = false;
        auto modCheck = [&](const char* name, std::uint32_t bit) {
            bool on = (curMask & bit) != 0u;
            if (ImGui::Checkbox(name, &on)) {
                if (on) curMask |= bit;
                else curMask &= ~bit;
                changed = true;
            }
        };
        modCheck("机载雷达 B-Scope / PPI（仅雷达机型）", kPanelRadar);
        modCheck("原生距离-多普勒（绿磷信号级 R-D）", kPanelSignal);
        modCheck("本机电子战发射与 RWR/ESM 截获", kPanelEw);
        modCheck("本机武器包线与外挂状态", kPanelWeapon);
        modCheck("本机 6-DOF 气动舵面与推进", kPanelAero);
        modCheck("本机数据链 / PNT / 融合航迹", kPanelComms);
        modCheck("左下角载具列表", kPanelFleet);
        if (changed) {
            controls_->customPanelMask.store(curMask, std::memory_order_relaxed);
            controls_->workspacePreset.store(static_cast<int>(WorkspacePreset::Custom), std::memory_order_relaxed);
        }
        ImGui::Separator();
        ImGui::TextDisabled("规划中真实面板（VIS-M15 接入真实物理流，绝不虚造）：");
        ImGui::BeginDisabled();
        bool f = false;
        ImGui::Checkbox("完整 RWR 威胁方位环 & 信号级回放录制 [VIS-M15]", &f);
        ImGui::Checkbox("EO/IR 热成像吊舱 & 导弹导引头实时画面 [VIS-M15]", &f);
        ImGui::Checkbox("SAR / 气象雷达 & SMS 质心 / 战争迷雾对比 [VIS-M15]", &f);
        ImGui::EndDisabled();
        ImGui::EndPopup();
    }

    // =========================================================================
    // CLUSTER 3: Viewport Environment Tools (Camera | 3-D Layers | Clock | Layout)
    // =========================================================================
    if (!compact) {
        drawVerticalHairline(s);
    }

    // Camera Mode Dropdown Pill + Reset
    int camMode = std::clamp(controls_->cameraMode.load(std::memory_order_relaxed), 0, world::CameraController::kModeCount - 1);
    const char* camShort[] = {"追踪", "环绕", "总览", "自由"};
    const char* camLong[] = {"追踪视角 (Chase)", "环绕视角 (Orbit)", "正北总览 (Overview)", "自由视角 (Free)"};
    char camBtn[40];
    std::snprintf(camBtn, sizeof camBtn, "%s ▼###cam_btn", camShort[camMode]);
    if (ImGui::Button(camBtn)) ImGui::OpenPopup("cam_mode_popup");
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("切换机位视角（快捷键 C 轮切）");
    if (ImGui::BeginPopup("cam_mode_popup")) {
        ImGui::TextDisabled("视角模式 [C]");
        ImGui::Separator();
        for (int i = 0; i < world::CameraController::kModeCount; ++i) {
            if (ImGui::Selectable(camLong[i], i == camMode)) {
                controls_->cameraMode.store(i, std::memory_order_relaxed);
            }
        }
        ImGui::EndPopup();
    }
    ImGui::SameLine(0.0f, 3.0f * s);
    if (ImGui::Button("重置")) controls_->cameraReset.store(true, std::memory_order_relaxed);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("重置相机视距与角度 [R]");

    // 3-D Tactical Overlays Popover
    std::uint32_t mask = controls_->tacticalOverlayMask.load(std::memory_order_relaxed);
    ImGui::SameLine(0.0f, 5.0f * s);
    if (mask != 0u) ImGui::PushStyleColor(ImGuiCol_Button, tint(0.11f, 0.39f, 0.50f, 0.88f));
    const char* layerLabel = mask == kOverlayAll ? "3-D 图层 · 全开 ▼"
                           : mask != 0u          ? "3-D 图层 · 部分 ▼"
                                                 : "3-D 图层 · 关 ▼";
    if (ImGui::Button(layerLabel)) ImGui::OpenPopup("tact_overlay_popup");
    if (mask != 0u) ImGui::PopStyleColor();

    if (ImGui::BeginPopup("tact_overlay_popup")) {
        ImGui::TextDisabled("3-D 态势图层（仅显示当前聚焦载具）");
        if (ImGui::SmallButton("全开")) mask = kOverlayAll;
        ImGui::SameLine();
        if (ImGui::SmallButton("全关 (0 ms)")) mask = 0u;
        ImGui::Separator();
        auto toggleBit = [&](const char* label, std::uint32_t bit, ImU32 swatchCol) {
            bool on = (mask & bit) != 0u;
            const ImVec2 p = ImGui::GetCursorScreenPos();
            const float dotR = 3.5f * s;
            const float cy = p.y + ImGui::GetFrameHeight() * 0.5f;
            ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(p.x + dotR, cy), dotR, swatchCol);
            ImGui::Dummy(ImVec2(dotR * 2.4f, 0.0f));
            ImGui::SameLine(0.0f, 4.0f * s);
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

    // Scene Solar Clock Popover (00:00..24:00 local solar time + presets + Real-Time UTC)
    const double reqSolar = controls_->solarLocalHours.load(std::memory_order_relaxed);
    const double shownSolar = controls_->activeSolarLocalHours.load(std::memory_order_relaxed);
    const double normH = std::fmod(std::max(0.0, shownSolar) + 24.0, 24.0);
    const int dispHr = static_cast<int>(normH) % 24;
    const int dispMin = static_cast<int>(std::floor((normH - std::floor(normH)) * 60.0 + 0.5)) % 60;
    const bool realClock = (reqSolar < -0.5 && reqSolar > -1.5);
    char clockBtn[48];
    if (realClock) std::snprintf(clockBtn, sizeof clockBtn, "时钟 实时 %02d:%02d ▼###solar_clk", dispHr, dispMin);
    else std::snprintf(clockBtn, sizeof clockBtn, "时钟 %02d:%02d ▼###solar_clk", dispHr, dispMin);
    ImGui::SameLine(0.0f, 5.0f * s);
    ImGui::PushStyleColor(ImGuiCol_Button, tint(0.32f, 0.25f, 0.11f, 0.86f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, tint(0.44f, 0.34f, 0.15f, 0.96f));
    if (ImGui::Button(clockBtn)) ImGui::OpenPopup("solar_clock_popup");
    ImGui::PopStyleColor(2);

    if (ImGui::BeginPopup("solar_clock_popup")) {
        const double sunEl = controls_->activeSunElevationDeg.load(std::memory_order_relaxed);
        ImGui::Text("场景光照时钟（当地太阳时）");
        ImGui::SameLine(0.0f, 10.0f * s);
        ImGui::TextColored(sunEl > 5.0 ? tint(1.0f, 0.84f, 0.36f) : tint(0.56f, 0.72f, 0.96f),
                           "高度角 %+.0f°", sunEl);
        float sliderH = static_cast<float>(normH);
        char fmtBuf[16];
        std::snprintf(fmtBuf, sizeof fmtBuf, "%02d:%02d", dispHr, dispMin);
        ImGui::SetNextItemWidth(250.0f * s);
        if (ImGui::SliderFloat("##solar_slider", &sliderH, 0.0f, 23.98f, fmtBuf)) {
            controls_->solarLocalHours.store(static_cast<double>(sliderH), std::memory_order_relaxed);
        }
        struct SolarPreset { const char* label; double hours; };
        static constexpr SolarPreset kPresets[] = {
            {"清晨", 7.5}, {"正午", 12.5}, {"午后", 14.5}, {"黄昏", 17.25}, {"夜晚", 22.0}, {"实时", -1.0},
        };
        for (int i = 0; i < 6; ++i) {
            if (i > 0) ImGui::SameLine(0.0f, 4.0f * s);
            const bool act = (kPresets[i].hours < 0.0) ? realClock
                                                       : (reqSolar >= 0.0 && std::abs(reqSolar - kPresets[i].hours) < 0.35);
            if (act) ImGui::PushStyleColor(ImGuiCol_Button, tint(0.18f, 0.46f, 0.76f, 0.96f));
            if (ImGui::SmallButton(kPresets[i].label)) {
                controls_->solarLocalHours.store(kPresets[i].hours, std::memory_order_relaxed);
            }
            if (act) ImGui::PopStyleColor();
        }
        ImGui::EndPopup();
    }

    // Panel Size & Layout Settings Popover (Point 6!)
    ImGui::SameLine(0.0f, 5.0f * s);
    if (ImGui::Button("布局 ▼###layout_btn")) ImGui::OpenPopup("layout_settings_popup");
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("调节小面板大小、字号缩放与右侧停靠宽度");

    if (ImGui::BeginPopup("layout_settings_popup")) {
        ImGui::Text("小面板大小与布局设置");
        ImGui::Separator();
        float pScale = std::clamp(controls_->panelScale.load(std::memory_order_relaxed), 0.75f, 1.35f);
        float dockW = std::clamp(controls_->rightDockWidthPx.load(std::memory_order_relaxed), 280.0f, 520.0f);

        struct SizePreset { const char* label; float sc; float w; };
        static constexpr SizePreset kSz[] = {
            {"紧凑 85%%", 0.85f, 320.0f},
            {"标准 100%%", 1.00f, 370.0f},
            {"宽松 118%%", 1.18f, 430.0f},
        };
        for (int i = 0; i < 3; ++i) {
            if (i > 0) ImGui::SameLine(0.0f, 5.0f * s);
            const bool act = std::abs(pScale - kSz[i].sc) < 0.03f;
            if (act) ImGui::PushStyleColor(ImGuiCol_Button, tint(0.18f, 0.46f, 0.76f, 0.96f));
            if (ImGui::SmallButton(kSz[i].label)) {
                pScale = kSz[i].sc;
                dockW = kSz[i].w;
                controls_->panelScale.store(pScale, std::memory_order_relaxed);
                controls_->rightDockWidthPx.store(dockW, std::memory_order_relaxed);
            }
            if (act) ImGui::PopStyleColor();
        }

        ImGui::TextDisabled("面板与字号缩放 (%.0f%%)", static_cast<double>(pScale * 100.0f));
        ImGui::SetNextItemWidth(240.0f * s);
        if (ImGui::SliderFloat("##panel_scale", &pScale, 0.75f, 1.35f, "%.2fx")) {
            controls_->panelScale.store(pScale, std::memory_order_relaxed);
        }
        ImGui::TextDisabled("右侧面板宽度 (%.0f px)", static_cast<double>(dockW));
        ImGui::SetNextItemWidth(240.0f * s);
        if (ImGui::SliderFloat("##dock_width", &dockW, 280.0f, 520.0f, "%.0f px")) {
            controls_->rightDockWidthPx.store(dockW, std::memory_order_relaxed);
        }
        ImGui::EndPopup();
    }

    // Demo-only compact pause/step inline (when NOT in mirror and NOT showing replay bottom bar)
    const double rStart = controls_->replayStart.load(std::memory_order_relaxed);
    const double rEnd = controls_->replayEnd.load(std::memory_order_relaxed);
    if (!source_.mirror && !(rEnd > rStart)) {
        drawVerticalHairline(s);
        bool paused = controls_->paused.load(std::memory_order_relaxed);
        if (ImGui::Button(paused ? "继续" : "暂停")) controls_->paused.store(!paused, std::memory_order_relaxed);
        ImGui::SameLine(0.0f, 3.0f * s);
        if (ImGui::Button("单步")) controls_->singleStep.store(true, std::memory_order_relaxed);
        if (!compact) {
            ImGui::SameLine(0.0f, 6.0f * s);
            ImGui::AlignTextToFramePadding();
            ImGui::TextDisabled("t=%.1fs", simTime);
        }
    }

    const ImVec2 winPos = ImGui::GetWindowPos();
    const ImVec2 winSz = ImGui::GetWindowSize();
    controls_->topBarBottomPx.store(winPos.y + winSz.y, std::memory_order_relaxed);
    controls_->topBarRightPx.store(winPos.x + winSz.x, std::memory_order_relaxed);

    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(5);
}

void MonitorGui::drawReplayBar() const {
    const double start = controls_->replayStart.load(std::memory_order_relaxed);
    const double end = controls_->replayEnd.load(std::memory_order_relaxed);
    if (source_.mirror || !(end > start)) {
        controls_->bottomBarTopPx.store(0.0f, std::memory_order_relaxed);
        return;
    }

    const Layout l = layout(controls_.get());
    const float s = l.scale;
    const float barW = std::min(560.0f * s, std::max(320.0f * s, l.display.x * 0.42f));
    const float leftR = controls_->leftDockRightPx.load(std::memory_order_relaxed);
    const float rightL = controls_->rightDockLeftPx.load(std::memory_order_relaxed);
    const float availLeft = leftR > 10.0f ? (leftR + 12.0f * s) : l.margin;
    const float availRight = (rightL > 10.0f && rightL < l.display.x - 10.0f) ? (rightL - 12.0f * s) : (l.display.x - l.margin);
    const float actualW = std::min(barW, std::max(240.0f * s, availRight - availLeft));
    const float centerX = (availLeft + availRight) * 0.5f;
    const float posX = std::clamp(centerX - actualW * 0.5f, availLeft, std::max(availLeft, availRight - actualW));
    const float barH = 38.0f * s;

    ImGui::SetNextWindowPos(ImVec2(posX, l.display.y - l.margin - barH), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(actualW, barH), ImGuiCond_Always);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 18.0f * s);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f * s, 6.0f * s));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 6.0f * s);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6.0f * s, 4.0f * s));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, tint(0.043f, 0.063f, 0.094f, 0.88f));
    ImGui::PushStyleColor(ImGuiCol_Border, tint(0.46f, 0.62f, 0.82f, 0.28f));

    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                   ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
    if (!beginMandatoryPanel("回放控制###replay_bar", flags)) {
        ImGui::End();
        ImGui::PopStyleColor(2);
        ImGui::PopStyleVar(4);
        return;
    }

    bool paused = controls_->paused.load(std::memory_order_relaxed);
    ImGui::PushStyleColor(ImGuiCol_Button, tint(0.14f, 0.46f, 0.78f, 0.94f));
    if (ImGui::Button(paused ? "继续" : "暂停")) {
        controls_->paused.store(!paused, std::memory_order_relaxed);
    }
    ImGui::PopStyleColor();
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("暂停 / 继续回放 [Space]");

    ImGui::SameLine(0.0f, 5.0f * s);
    if (ImGui::Button("单步")) controls_->singleStep.store(true, std::memory_order_relaxed);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("单步前进一帧 [.]");

    ImGui::SameLine(0.0f, 8.0f * s);
    const double simTime = controls_->simTime.load(std::memory_order_relaxed);
    char tBuf[40];
    std::snprintf(tBuf, sizeof tBuf, "%.1fs / %.1fs", simTime, end);
    const float readoutW = ImGui::CalcTextSize(tBuf).x + 12.0f * s;
    const float sliderW = std::max(80.0f * s, ImGui::GetContentRegionAvail().x - readoutW);
    float at = static_cast<float>(simTime);
    ImGui::SetNextItemWidth(sliderW);
    if (ImGui::SliderFloat("##replay_timeline", &at, static_cast<float>(start), static_cast<float>(end), "")) {
        controls_->seekTo.store(static_cast<double>(at), std::memory_order_relaxed);
    }
    ImGui::SameLine(0.0f, 8.0f * s);
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("%s", tBuf);
    if (ImGui::IsItemHovered() && !aircraft_.empty()) ImGui::SetTooltip("%s", aircraft_.c_str());

    controls_->bottomBarTopPx.store(ImGui::GetWindowPos().y, std::memory_order_relaxed);
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(4);
}

void MonitorGui::drawStatus(const sim::VehicleState& s, const ipc::FlightStatus& status, bool state) const {
    const double* d = status.derived;
    if (!(d[ipc::kDerivedMassKg] > 0.0)) return;
    const float gap = 10.0f * uiScale();
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

    const Layout l = layout(controls_.get());
    ImGui::SetNextWindowPos(ImVec2(l.margin, l.listTop), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(l.listWidth, l.listHeight), ImGuiCond_Always);
    if (!beginMandatoryPanel("载具列表###vehicles", ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::End();
        return;
    }
    ImGui::SetWindowFontScale(l.panelScale);

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
        ImGui::TableSetupColumn("海高 m");
        ImGui::TableSetupColumn("离地 m");
        ImGui::TableSetupColumn("空速");
        ImGui::TableSetupColumn("航向");
        ImGui::TableSetupColumn("状态");
        ImGui::TableHeadersRow();

        for (std::size_t i = 0; i < batch_->states.size(); ++i) {
            if (i < vehicles_.size() && !vehicles_[i].alive) continue;
            const auto& s = batch_->states[i];
            const bool isSelected = (static_cast<int>(i) == selected);
            const char* word = "";
            const int level =
                status_ && i < status_->size() && !s.onGround && !s.crashed ? severity((*status_)[i], &word) : 0;

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            char label[80];
            if (i < vehicles_.size() && !vehicles_[i].name.empty()) std::snprintf(label, sizeof label, "%s", vehicles_[i].name.c_str());
            else std::snprintf(label, sizeof label, "v%zu", i);
            char selectableId[96];
            std::snprintf(selectableId, sizeof selectableId, "%s##veh_%zu", label, i);
            if (ImGui::Selectable(selectableId, isSelected, ImGuiSelectableFlags_SpanAllColumns))
                controls_->selectedVehicle.store(static_cast<int>(i), std::memory_order_relaxed);
            if (level) ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1,
                                              level >= 2 ? colour(165, 32, 32, 100) : colour(180, 130, 20, 85));

            ImGui::TableSetColumnIndex(1);
            ImGui::TextDisabled("%s", i < vehicles_.size() && vehicles_[i].level ? vehicles_[i].level : "");
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.0f", s.altitudeMslM);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.0f", s.altitudeAglM);
            ImGui::TableSetColumnIndex(4);
            ImGui::Text("%.0f", s.airspeedTrueMs);
            ImGui::TableSetColumnIndex(5);
            ImGui::Text("%.0f°", std::fmod(units::radiansToDegrees(s.eulerRad[2]) + 360.0, 360.0));
            ImGui::TableSetColumnIndex(6);
            if (s.diverged) ImGui::TextColored(warningText(), "发散");
            else if (s.crashed) ImGui::TextColored(warningText(), "坠毁");
            else if (level) ImGui::TextColored(level >= 2 ? warningText() : cautionText(), "%s", word);
            else ImGui::TextUnformatted(s.onGround ? "地面" : "空中");
        }
        ImGui::EndTable();
    }

    const ImVec2 winPos = ImGui::GetWindowPos();
    const ImVec2 winSz = ImGui::GetWindowSize();
    controls_->leftDockRightPx.store(winPos.x + winSz.x, std::memory_order_relaxed);
    controls_->leftDockTopPx.store(winPos.y, std::memory_order_relaxed);
    ImGui::End();
}

} // namespace fsim::ui
