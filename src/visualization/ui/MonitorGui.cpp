#include "visualization/ui/MonitorGui.h"
#include "visualization/ui/MandatoryPanel.h"

#include "foundation/core/Units.h"
#include "visualization/ui/Colour.h"
#include "visualization/scene/CameraController.h"

#include <vsgImGui/imgui.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>

namespace fsim::ui {

namespace {

/// Panels are laid out against the window that exists now, not against fixed
/// pixels: the vehicle list stays anchored cleanly at the bottom-left and the
/// monitor stops above it so they never collide at any DPI or aspect ratio.
struct Layout {
    float scale;  ///< effective UI scale factor (native TTF size or DPI scale)
    float margin; ///< gap to the window edge and between panels
    ImVec2 display;
    float panelWidth;  ///< monitor width: spacious enough for Chinese labels, never half the screen
    float listWidth, listHeight, listTop; ///< vehicle list anchored to bottom-left
    float monitorMaxHeight;               ///< keeps monitor strictly above vehicle list
};

Layout layout() {
    Layout l;
    l.scale = uiScale();
    l.margin = 10.0f * l.scale;
    l.display = ImGui::GetIO().DisplaySize;
    // B-43: Fit cleanly inside 1600x900 @ 150% DPI as well as 3840x2160 native 4K.
    l.panelWidth = std::min(340.0f * l.scale, std::max(260.0f, l.display.x * 0.34f));
    l.listWidth = std::min(480.0f * l.scale, std::max(280.0f, l.display.x * 0.46f));
    l.listHeight = std::min(220.0f * l.scale, std::max(100.0f, l.display.y * 0.25f));
    l.listTop = std::max(l.margin, l.display.y - l.listHeight - l.margin);
    l.monitorMaxHeight = std::max(140.0f, l.listTop - 2.0f * l.margin);
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
    // Background draw list: labels sit under the panels, over the 3D scene.
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const float k = uiScale();
    for (const Label& l : labels_) {
        const ImVec2 size = ImGui::CalcTextSize(l.text.c_str());
        const ImVec2 pos(l.x + 10.0f * k, l.y - size.y * 0.5f);
        const ImU32 bg = l.severity >= 2 ? colour(196, 40, 36, 238)
                       : l.severity == 1 ? colour(232, 170, 30, 238)
                       : l.selected      ? colour(216, 128, 24, 232)
                                         : colour(16, 24, 38, 212);
        const ImU32 border = l.selected ? colour(255, 224, 150, 225) : colour(96, 126, 162, 110);
        const ImVec2 lo(pos.x - 5.0f * k, pos.y - 2.5f * k);
        const ImVec2 hi(pos.x + size.x + 5.0f * k, pos.y + size.y + 2.5f * k);
        dl->AddRectFilled(lo, hi, bg, 3.5f * k);
        dl->AddRect(lo, hi, border, 3.5f * k, 0, (l.selected ? 1.4f : 0.9f) * k);
        dl->AddLine(ImVec2(l.x, l.y), ImVec2(lo.x, l.y), bg, 1.4f * k);
        dl->AddText(pos, l.severity == 1 ? colour(18, 14, 0, 255) : colour(245, 249, 255, 255), l.text.c_str());
    }
}

void MonitorGui::drawMonitor() const {
    const Layout l = layout();
    ImGui::SetNextWindowPos(ImVec2(l.margin, l.margin), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(l.panelWidth, 0.0f), ImVec2(l.panelWidth, l.monitorMaxHeight));
    if (!beginMandatoryPanel("飞行监控###flightsim", ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }

    const double fps = controls_->fps.load(std::memory_order_relaxed);
    const double frameMs = controls_->frameMs.load(std::memory_order_relaxed);
    const double sps = controls_->simThroughput.load(std::memory_order_relaxed);
    const double simTime = controls_->simTime.load(std::memory_order_relaxed);
    const bool compact = (l.display.y / l.scale) <= 720.0f;
    const ImGuiTreeNodeFlags openUnlessCompact = compact ? ImGuiTreeNodeFlags_None : ImGuiTreeNodeFlags_DefaultOpen;

    std::size_t live = 0;
    if (!vehicles_.empty()) {
        for (const auto& v : vehicles_) live += v.alive ? 1 : 0;
    } else if (batch_) {
        live = batch_->states.size();
    }

    // Line 1: Prominent, unmistakable FPS & frame time header (never pushed off-screen by long filenames).
    const ImVec4 fpsCol = fps >= 55.0 ? tint(0.32f, 0.92f, 0.56f)
                        : fps >= 30.0 ? cautionText()
                                      : warningText();
    ImGui::TextColored(fpsCol, "帧率 %.0f FPS", fps);
    ImGui::SameLine();
    ImGui::TextDisabled("(%.1f ms)  ·  %zu 架", frameMs, live);

    // Line 2: Active source / replay clock & playback controls.
    if (source_.mirror) {
        if (!source_.attached) {
            ImGui::TextColored(cautionText(), "等待仿真程序接入...");
        } else {
            ImGui::Text("世界: %s", source_.world.c_str());
        }
        ImGui::TextDisabled("时间 %.1f s  ·  %.0f 步/秒", source_.simTime, source_.vehicleStepsPerSecond);
    } else {
        if (!aircraft_.empty()) ImGui::TextDisabled("%s", aircraft_.c_str());
        if (sps > 0.5) ImGui::Text("时间 %.1f s  ·  %.0f 步/秒  ·  1x", simTime, sps);
        else ImGui::Text("时间 %.1f s  ·  1x 实时", simTime);

        bool paused = controls_->paused.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("暂停 [Space]", &paused)) controls_->paused.store(paused, std::memory_order_relaxed);
        ImGui::SameLine();
        if (ImGui::SmallButton("单步 [.]")) controls_->singleStep.store(true, std::memory_order_relaxed);
        const double start = controls_->replayStart.load(std::memory_order_relaxed);
        const double end = controls_->replayEnd.load(std::memory_order_relaxed);
        if (end > start) {
            float at = static_cast<float>(simTime);
            ImGui::SetNextItemWidth(-1.0f);
            if (ImGui::SliderFloat("##timeline", &at, static_cast<float>(start), static_cast<float>(end), "%.1f s"))
                controls_->seekTo.store(static_cast<double>(at), std::memory_order_relaxed);
        }
    }

    ImGui::Separator();

    // 1. Camera (clean 4 modes)
    if (ImGui::CollapsingHeader("视角控制  [C]", ImGuiTreeNodeFlags_DefaultOpen)) {
        int mode = std::clamp(controls_->cameraMode.load(std::memory_order_relaxed), 0, world::CameraController::kModeCount - 1);
        const char* modes[] = {"追踪视角", "环绕视角（正北）", "俯视总览", "自由视角"};
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::Combo("##camera", &mode, modes, world::CameraController::kModeCount))
            controls_->cameraMode.store(mode, std::memory_order_relaxed);
        if (ImGui::SmallButton("拉远")) controls_->cameraZoom.store(1.25, std::memory_order_relaxed);
        ImGui::SameLine();
        if (ImGui::SmallButton("拉近")) controls_->cameraZoom.store(0.8, std::memory_order_relaxed);
        ImGui::SameLine();
        if (ImGui::SmallButton("重置 [R]")) controls_->cameraReset.store(true, std::memory_order_relaxed);
        if (!compact) {
            ImGui::TextDisabled("视点 %.4f, %.4f  ·  %.0f m",
                                controls_->eyeLatDeg.load(std::memory_order_relaxed),
                                controls_->eyeLonDeg.load(std::memory_order_relaxed),
                                controls_->eyeDistanceM.load(std::memory_order_relaxed));
        }
    }

    // 2. Tactical & EMSO Overlays (strictly filtered to the currently focused vehicle)
    if (ImGui::CollapsingHeader("态势图层（仅聚焦载具）", openUnlessCompact)) {
        std::uint32_t mask = controls_->tacticalOverlayMask.load(std::memory_order_relaxed);
        if (ImGui::SmallButton("全开")) mask = kOverlayAll;
        ImGui::SameLine();
        if (ImGui::SmallButton("全关")) mask = 0u;
        ImGui::SameLine();
        ImGui::TextDisabled("%s", mask ? "已开启" : "已关闭 (0 ms)");

        auto toggleBit = [&](const char* label, std::uint32_t bit, ImU32 swatchCol) {
            bool on = (mask & bit) != 0u;
            const ImVec2 p = ImGui::GetCursorScreenPos();
            const float dotR = 3.6f * l.scale;
            const float cy = p.y + ImGui::GetFrameHeight() * 0.5f;
            ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(p.x + dotR, cy), dotR, swatchCol);
            ImGui::Dummy(ImVec2(dotR * 2.5f, 0.0f));
            ImGui::SameLine(0.0f, 3.0f * l.scale);
            if (ImGui::Checkbox(label, &on)) {
                if (on) mask |= bit;
                else mask &= ~bit;
            }
        };
        if (ImGui::BeginTable("tact_layers", 2, ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); toggleBit("雷达", kOverlayRadar, colour(46, 174, 255, 255));
            ImGui::TableSetColumnIndex(1); toggleBit("电子干扰", kOverlayJammer, colour(198, 86, 250, 255));
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); toggleBit("电子侦察", kOverlayEsm, colour(255, 204, 52, 255));
            ImGui::TableSetColumnIndex(1); toggleBit("数据链", kOverlayLinks, colour(46, 224, 158, 255));
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); toggleBit("武器射程", kOverlayWeapons, colour(42, 214, 122, 255));
            ImGui::TableSetColumnIndex(1); toggleBit("SAR / 气象", kOverlaySwaths, colour(56, 204, 232, 255));
            ImGui::EndTable();
        }
        controls_->tacticalOverlayMask.store(mask, std::memory_order_relaxed);
    }

    // 3. Focused Vehicle Telemetry
    if (batch_ && !batch_->states.empty()) {
        if (ImGui::CollapsingHeader("当前聚焦  [Tab]", openUnlessCompact)) {
            const int sel = std::clamp(controls_->selectedVehicle.load(std::memory_order_relaxed), 0,
                                       static_cast<int>(batch_->states.size()) - 1);
            const auto& s = batch_->states[static_cast<std::size_t>(sel)];
            const auto* meta = static_cast<std::size_t>(sel) < vehicles_.size() ? &vehicles_[static_cast<std::size_t>(sel)] : nullptr;
            if (meta && meta->alive) {
                ImGui::TextColored(tint(0.42f, 0.84f, 1.0f), "%s%s", meta->name.c_str(), s.diverged ? "  发散" : "");
            } else {
                ImGui::TextColored(tint(0.42f, 0.84f, 1.0f), "载具 %d%s", sel, s.diverged ? "  发散" : "");
            }
            ImGui::Text("海高 %.0f m  ·  离地 %.0f m", s.altitudeMslM, s.altitudeAglM);
            ImGui::Text("空速 %.0f m/s (%.0f 节)  ·  迎角 %.1f°", s.airspeedTrueMs,
                        units::metresPerSecondToKnots(s.airspeedTrueMs), units::radiansToDegrees(s.alphaRad));
            ImGui::Text("滚转 %.0f°  ·  俯仰 %.0f°  ·  航向 %.0f°", units::radiansToDegrees(s.eulerRad[0]),
                        units::radiansToDegrees(s.eulerRad[1]), units::radiansToDegrees(s.eulerRad[2]));

            // Expandable native 6-DOF aerodynamics, flight-control & propulsion details (collapsed by default)
            if (ImGui::TreeNode("气动与动力底层参数")) {
                ImGui::Text("马赫 %.2f · 表速 %.0f m/s · 过载 %.2f G", s.mach, s.airspeedCalibratedMs, s.loadFactor);
                ImGui::Text("侧滑 %.1f° · 角速率 %+.0f/%+.0f/%+.0f °/s",
                            units::radiansToDegrees(s.betaRad),
                            units::radiansToDegrees(s.angularRateBodyRadS[0]),
                            units::radiansToDegrees(s.angularRateBodyRadS[1]),
                            units::radiansToDegrees(s.angularRateBodyRadS[2]));
                ImGui::TextDisabled("舵面: 副翼 %+.1f° · 升降 %+.1f° · 方向 %+.1f°",
                                    units::radiansToDegrees(s.aileronRad),
                                    units::radiansToDegrees(s.elevatorRad),
                                    units::radiansToDegrees(s.rudderRad));
                double totalThrustKn = 0.0;
                double maxN2 = 0.0;
                double maxAb = 0.0;
                const int engN = std::clamp(s.engineCount, 0, sim::VehicleState::kMaxEngines);
                for (int e = 0; e < engN; ++e) {
                    totalThrustKn += s.thrustN[e] * 1e-3;
                    maxN2 = std::max(maxN2, s.engineN2[e]);
                    maxAb = std::max(maxAb, s.afterburner[e]);
                }
                if (engN > 0) {
                    ImGui::Text("推力 %.1f kN · 核心 %.0f%% · 加力 %.0f%%", totalThrustKn, maxN2, maxAb * 100.0);
                }
                if (std::isfinite(s.fuelKg) && s.fuelKg > 0.0) {
                    ImGui::TextDisabled("剩余燃油 %.0f kg · 减速板 %.0f%%", s.fuelKg, static_cast<double>(s.speedbrakePosition) * 100.0);
                }
                ImGui::TreePop();
            }
        }
    }

    // 4. World Connection & Weather (mirror mode only)
    if (source_.mirror && ImGui::CollapsingHeader("世界与环境", compact ? ImGuiTreeNodeFlags_None : ImGuiTreeNodeFlags_DefaultOpen)) {
        if (!source_.attached && !source_.available.empty()) {
            ImGui::Text("可接入世界:");
            for (std::size_t i = 0; i < source_.available.size(); ++i) {
                ImGui::PushID(static_cast<int>(i));
                if (ImGui::SmallButton("接入")) controls_->attachWorld.store(static_cast<int>(i), std::memory_order_relaxed);
                ImGui::PopID();
                ImGui::SameLine();
                ImGui::Text("%s", source_.available[i].c_str());
            }
        } else if (source_.attached) {
            if (source_.available.size() > 1) {
                int current = -1;
                for (std::size_t i = 0; i < source_.available.size(); ++i)
                    if (source_.available[i] == source_.world) current = static_cast<int>(i);
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::BeginCombo("切换世界", current >= 0 ? source_.available[static_cast<std::size_t>(current)].c_str() : source_.world.c_str())) {
                    for (std::size_t i = 0; i < source_.available.size(); ++i) {
                        const bool selected = static_cast<int>(i) == current;
                        if (ImGui::Selectable(source_.available[i].c_str(), selected) && !selected)
                            controls_->attachWorld.store(static_cast<int>(i), std::memory_order_relaxed);
                    }
                    ImGui::EndCombo();
                }
            }
            if (!source_.publisherAlive) ImGui::TextColored(warningText(), "仿真进程已退出");
            else if (source_.ageSeconds > 2.0) ImGui::TextColored(cautionText(), "暂未更新 (%.0f s)", source_.ageSeconds);
            else ImGui::TextDisabled("实时同步 · 延迟 %.0f ms", source_.ageSeconds * 1e3);
            if (!source_.environment.empty()) ImGui::TextWrapped("%s", source_.environment.c_str());
        }
    }
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
            ImGui::SetTooltip("当前空速允许的最大转弯坡度（留有 1.3 失速裕度）");
    }
}

void MonitorGui::drawVehicleList() const {
    if (!batch_) return;
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
        ImGui::TableSetupColumn("名称", ImGuiTableColumnFlags_WidthStretch, 1.7f);
        ImGui::TableSetupColumn("控制", ImGuiTableColumnFlags_WidthStretch, 1.1f);
        ImGui::TableSetupColumn("海高 m", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("离地 m", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("空速", ImGuiTableColumnFlags_WidthStretch, 0.9f);
        ImGui::TableSetupColumn("航向", ImGuiTableColumnFlags_WidthStretch, 0.8f);
        ImGui::TableSetupColumn("状态", ImGuiTableColumnFlags_WidthStretch, 1.2f);
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
    ImGui::End();
}

} // namespace fsim::ui
