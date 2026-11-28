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
/// pixels: the vehicle list used to be pinned 440 px down and 320 px tall,
/// which falls off the bottom of any window shorter than 760 px.
struct Layout {
    float scale;  ///< DPI scale the app applied to the font and style
    float margin; ///< gap to the window edge and between panels
    ImVec2 display;
    float panelWidth;  ///< monitor width: wide enough to read, never half the screen
    float listWidth, listHeight, listTop; ///< the vehicle list, anchored to the bottom
    float monitorMaxHeight;               ///< so the monitor stops above the list rather than under it
};

Layout layout() {
    Layout l;
    l.scale = std::max(ImGui::GetIO().FontGlobalScale, 0.5f);
    l.margin = 10.0f * l.scale;
    l.display = ImGui::GetIO().DisplaySize;
    // The text is sized by the DPI scale, so that is what the panel has to fit;
    // the window fraction is only a ceiling for windows too narrow to give it.
    // B-43: Keep width and vertical split well inside 1600x900 @ 150% DPI (1066x600 logical).
    l.panelWidth = std::min(315.0f * l.scale, std::max(250.0f, l.display.x * 0.34f));
    l.listWidth = std::min(460.0f * l.scale, std::max(270.0f, l.display.x * 0.48f));
    l.listHeight = std::min(220.0f * l.scale, std::max(96.0f, l.display.y * 0.25f));
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
    if (c & ipc::kConstraintUpset) say = "UPSET", level = 2;
    else if (c & ipc::kConstraintDeviating) say = "DEVIATING", level = 2;
    else if (c & ipc::kConstraintLowEnergy) say = "LOW ENERGY", level = 1;
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
    const float k = ImGui::GetIO().FontGlobalScale;
    for (const Label& l : labels_) {
        const ImVec2 size = ImGui::CalcTextSize(l.text.c_str());
        const ImVec2 pos(l.x + 10.0f * k, l.y - size.y * 0.5f);
        // (a vehicle in an abnormal state: its label red, or a caution's amber - the selected one's ringed, as its own
        // orange would hide which it is)
        // (these two as picked, whatever the target: a warning's red under white, a caution's amber under black)
        const ImU32 bg = l.severity >= 2 ? colour(196, 40, 36, 235) : l.severity == 1 ? colour(232, 170, 30, 235)
                         : l.selected ? IM_COL32(230, 140, 30, 200) : IM_COL32(20, 30, 50, 170);
        const ImVec2 lo(pos.x - 4.0f * k, pos.y - 2.0f * k), hi(pos.x + size.x + 4.0f * k, pos.y + size.y + 2.0f * k);
        dl->AddRectFilled(lo, hi, bg, 3.0f * k);
        if (l.severity > 0 && l.selected) dl->AddRect(lo, hi, IM_COL32(255, 255, 255, 235), 3.0f * k, 0, 1.5f * k);
        dl->AddLine(ImVec2(l.x, l.y), ImVec2(pos.x - 4.0f * k, l.y), bg, 1.5f * k);
        dl->AddText(pos, l.severity == 1 ? IM_COL32(18, 14, 0, 255) : IM_COL32(255, 255, 255, 255), l.text.c_str());
    }
}

void MonitorGui::drawMonitor() const {
    const Layout l = layout();
    ImGui::SetNextWindowPos(ImVec2(l.margin, l.margin), ImGuiCond_FirstUseEver);
    // Height 0: ImGui fits it to the content, so nothing is ever cut off. The
    // constraint stops that fit where the vehicle list begins; past it the
    // panel scrolls instead of growing under the list.
    ImGui::SetNextWindowSize(ImVec2(l.panelWidth, 0.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(l.panelWidth * 0.6f, 0.0f), ImVec2(FLT_MAX, l.monitorMaxHeight));
    if (!beginMandatoryPanel("flightsim")) {
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

    // Compact summary header (always visible without clipping even at 1600x900 @ 150% DPI)
    if (source_.mirror) {
        if (!source_.attached) {
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "waiting for a training application...");
        } else {
            ImGui::Text("world '%s'  x %zu", source_.world.c_str(), live);
        }
        ImGui::Text("%.0f fps (%.1f ms) | t=%.1fs | %.0f st/s", fps, frameMs, source_.simTime, source_.vehicleStepsPerSecond);
    } else {
        ImGui::Text("%s  x %zu  |  %.0f fps (%.1f ms)", aircraft_.c_str(), live, fps, frameMs);
        ImGui::Text("t=%.1fs | %.0f st/s | 1x", simTime, sps);
        bool paused = controls_->paused.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("pause [spc]", &paused)) controls_->paused.store(paused, std::memory_order_relaxed);
        ImGui::SameLine();
        if (ImGui::SmallButton("step [.]")) controls_->singleStep.store(true, std::memory_order_relaxed);
        const double start = controls_->replayStart.load(std::memory_order_relaxed);
        const double end = controls_->replayEnd.load(std::memory_order_relaxed);
        if (end > start) {
            float at = static_cast<float>(simTime);
            ImGui::SetNextItemWidth(-1.0f);
            if (ImGui::SliderFloat("##timeline", &at, static_cast<float>(start), static_cast<float>(end), "%.1f s"))
                controls_->seekTo.store(static_cast<double>(at), std::memory_order_relaxed);
        }
    }

    // 1. Camera & Director Suite (always open)
    if (ImGui::CollapsingHeader("Camera & Director  [C / I]", ImGuiTreeNodeFlags_DefaultOpen)) {
        int mode = std::clamp(controls_->cameraMode.load(std::memory_order_relaxed), 0, world::CameraController::kModeCount - 1);
        const char* modes[] = {"chase", "orbit (north-up)", "overview", "free", "weapon (director)", "cockpit (HUD)"};
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::Combo("##camera", &mode, modes, world::CameraController::kModeCount))
            controls_->cameraMode.store(mode, std::memory_order_relaxed);
        if (ImGui::SmallButton("-")) controls_->cameraZoom.store(1.25, std::memory_order_relaxed);
        ImGui::SameLine();
        if (ImGui::SmallButton("+")) controls_->cameraZoom.store(0.8, std::memory_order_relaxed);
        ImGui::SameLine();
        if (ImGui::SmallButton("reset [R]")) controls_->cameraReset.store(true, std::memory_order_relaxed);
        ImGui::SameLine();
        bool pip = controls_->pipEnabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("PiP [I]", &pip)) controls_->pipEnabled.store(pip, std::memory_order_relaxed);
        if (!compact) {
            ImGui::Text("eye %.4f %.4f | %.0f m", controls_->eyeLatDeg.load(std::memory_order_relaxed),
                        controls_->eyeLonDeg.load(std::memory_order_relaxed),
                        controls_->eyeDistanceM.load(std::memory_order_relaxed));
        }
    }

    // 2. Tactical & EMSO 3-D Overlays (VIS-M14, 0.0 ms when OFF)
    if (ImGui::CollapsingHeader("Tactical & EMSO Overlays  [O]", openUnlessCompact)) {
        std::uint32_t mask = controls_->tacticalOverlayMask.load(std::memory_order_relaxed);
        if (ImGui::SmallButton("All")) mask = kOverlayAll;
        ImGui::SameLine();
        if (ImGui::SmallButton("None")) mask = 0u;
        ImGui::SameLine();
        ImGui::TextDisabled("%s", mask ? "ON" : "OFF (0 ms)");

        auto toggleBit = [&](const char* label, std::uint32_t bit) {
            bool on = (mask & bit) != 0u;
            if (ImGui::Checkbox(label, &on)) {
                if (on) mask |= bit;
                else mask &= ~bit;
            }
        };
        if (ImGui::BeginTable("tact_layers", 2, ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); toggleBit("Radar / STT", kOverlayRadar);
            ImGui::TableSetColumnIndex(1); toggleBit("EA / Burn-Thru", kOverlayJammer);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); toggleBit("ESM / CRLB", kOverlayEsm);
            ImGui::TableSetColumnIndex(1); toggleBit("Link16 / IFF", kOverlayLinks);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); toggleBit("DLZ / LCOS", kOverlayWeapons);
            ImGui::TableSetColumnIndex(1); toggleBit("SAR / Weather", kOverlaySwaths);
            ImGui::EndTable();
        }
        controls_->tacticalOverlayMask.store(mask, std::memory_order_relaxed);
    }

    // 3. Selected Vehicle Telemetry
    if (batch_ && !batch_->states.empty()) {
        if (ImGui::CollapsingHeader("Selected Vehicle  [Tab]", openUnlessCompact)) {
            const int sel = std::clamp(controls_->selectedVehicle.load(std::memory_order_relaxed), 0,
                                       static_cast<int>(batch_->states.size()) - 1);
            const auto& s = batch_->states[static_cast<std::size_t>(sel)];
            const auto* meta = static_cast<std::size_t>(sel) < vehicles_.size() ? &vehicles_[static_cast<std::size_t>(sel)] : nullptr;
            if (meta && meta->alive) ImGui::Text("%s%s", meta->name.c_str(), s.diverged ? "  DIVERGED" : "");
            else ImGui::Text("vehicle %d%s", sel, s.diverged ? "  DIVERGED" : "");
            ImGui::Text("alt %.0f m  agl %.0f m", s.altitudeMslM, s.altitudeAglM);
            ImGui::Text("tas %.0f m/s (%.0f kt)  aoa %.1f", s.airspeedTrueMs,
                        units::metresPerSecondToKnots(s.airspeedTrueMs), units::radiansToDegrees(s.alphaRad));
            ImGui::Text("roll %.0f  pitch %.0f  hdg %.0f", units::radiansToDegrees(s.eulerRad[0]),
                        units::radiansToDegrees(s.eulerRad[1]), units::radiansToDegrees(s.eulerRad[2]));
        }
    }

    // 4. World Connection & Environment (collapsible to save vertical space on high-DPI displays)
    if (source_.mirror && ImGui::CollapsingHeader("World & Weather", compact ? ImGuiTreeNodeFlags_None : ImGuiTreeNodeFlags_DefaultOpen)) {
        if (!source_.attached && !source_.available.empty()) {
            ImGui::Text("worlds in the registry:");
            for (std::size_t i = 0; i < source_.available.size(); ++i) {
                ImGui::PushID(static_cast<int>(i));
                if (ImGui::SmallButton("attach")) controls_->attachWorld.store(static_cast<int>(i), std::memory_order_relaxed);
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
                if (ImGui::BeginCombo("switch world", current >= 0 ? source_.available[static_cast<std::size_t>(current)].c_str() : source_.world.c_str())) {
                    for (std::size_t i = 0; i < source_.available.size(); ++i) {
                        const bool selected = static_cast<int>(i) == current;
                        if (ImGui::Selectable(source_.available[i].c_str(), selected) && !selected)
                            controls_->attachWorld.store(static_cast<int>(i), std::memory_order_relaxed);
                    }
                    ImGui::EndCombo();
                }
            }
            if (!source_.publisherAlive) ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "training application has exited");
            else if (source_.ageSeconds > 2.0) ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "not updating (%.0f s)", source_.ageSeconds);
            else ImGui::TextDisabled("live, last update %.0f ms ago", source_.ageSeconds * 1e3);
            if (!source_.environment.empty()) ImGui::TextWrapped("%s", source_.environment.c_str());
        }
    }
    ImGui::End();
}

void MonitorGui::drawStatus(const sim::VehicleState& s, const ipc::FlightStatus& status, bool state) const {
    // What its Vehicle Interface says of it (docs/vehicle-interface.md, 7.3), as items along a line (onto the next where
    // the panel is narrow): nothing from a world that publishes none.
    const double* d = status.derived;
    if (!(d[ipc::kDerivedMassKg] > 0.0)) return;
    const float gap = 14.0f * ImGui::GetIO().FontGlobalScale;
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
    if (state && flying) { // (the list's state column says it, where the list is shown)
        const char* word = "";
        if (const int level = severity(status, &word)) {
            std::snprintf(text, sizeof text, "%s", word);
            item(level >= 2 ? Tone::Warning : Tone::Caution);
        }
    }
    const double mass = d[ipc::kDerivedMassKg];
    std::snprintf(text, sizeof text, mass < 100.0 ? "%.2f kg" : "%.0f kg", mass);
    if (item(Tone::Plain)) {
        if (std::isfinite(s.fuelKg) && s.fuelKg > 0.0) ImGui::SetTooltip("what it weighs now, with %.0f kg of fuel in its tanks", s.fuelKg);
        else ImGui::SetTooltip("what it weighs now");
    }
    const double stall = d[ipc::kDerivedStallCasMs], least = d[ipc::kDerivedLeastCasMs], floor = d[ipc::kDerivedFloorCasMs];
    if (!(stall > 0.0) || !(least > 0.0)) return; // (a rotorcraft: no least airspeed)
    // a wing: its airspeed against the least a command may ask and where it stalls, at the weight it has
    const double cas = s.airspeedCalibratedMs;
    std::snprintf(text, sizeof text, "cas %.0f", cas);
    if (item(flying && cas < stall ? Tone::Warning : flying && cas < least ? Tone::Caution : Tone::Plain))
        ImGui::SetTooltip("its calibrated airspeed, m/s");
    std::snprintf(text, sizeof text, "least %.0f", least);
    if (item(Tone::Dim)) ImGui::SetTooltip("the least calibrated airspeed a command may ask, m/s:\n1.1 times its stall at the weight it has");
    std::snprintf(text, sizeof text, "stall %.0f", stall);
    if (item(Tone::Dim))
        ImGui::SetTooltip("its least calibrated airspeed at the weight it has, m/s\n(%.3f times what it is as it spawns)",
                          d[ipc::kDerivedLoadRatio] > 0.0 ? d[ipc::kDerivedLoadRatio] : 1.0);
    if (flying && floor > 0.0) {
        std::snprintf(text, sizeof text, "floor %.0f", floor);
        if (item(Tone::Caution)) ImGui::SetTooltip("its speed floor, m/s: it sank at the airspeed commanded,\nand is flown faster to keep its height");
    }
    // the bank its airspeed leaves a turn, while that is less than a bank it might be asked
    const double cap = d[ipc::kDerivedBankCapRad];
    if (flying && cap > 0.0 && cap < 1.05) {
        const double bank = std::abs(units::radiansToDegrees(s.eulerRad[0])), most = units::radiansToDegrees(cap);
        std::snprintf(text, sizeof text, "bank <= %.0f", most);
        if (item(bank > most + 5.0 ? Tone::Caution : Tone::Dim))
            ImGui::SetTooltip("the steepest bank its airspeed leaves a turn, deg:\na margin of 1.3 over the load factor it would stall at");
    }
}

void MonitorGui::drawVehicleList() const {
    if (!batch_) return;
    const Layout l = layout();
    // Anchored to the bottom-left of the window that exists, and never taller
    // than a third of it, so the whole table is always on screen.
    ImGui::SetNextWindowPos(ImVec2(l.margin, l.listTop), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(l.listWidth, l.listHeight), ImGuiCond_FirstUseEver);
    if (!beginMandatoryPanel("vehicles")) {
        ImGui::End();
        return;
    }
    const int selected = controls_->selectedVehicle.load(std::memory_order_relaxed);
    // the selected vehicle's flight status, a line above the table
    if (status_ && selected >= 0 && static_cast<std::size_t>(selected) < status_->size() &&
        static_cast<std::size_t>(selected) < batch_->states.size() &&
        (static_cast<std::size_t>(selected) >= vehicles_.size() || vehicles_[static_cast<std::size_t>(selected)].alive))
        drawStatus(batch_->states[static_cast<std::size_t>(selected)], (*status_)[static_cast<std::size_t>(selected)], false);
    // Stretch rather than fixed-fit: fixed columns overflow a narrow panel and
    // ImGui hides the overflow behind "...", which loses the right-hand values.
    const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY |
                                  ImGuiTableFlags_SizingStretchProp;
    if (ImGui::BeginTable("vehicles", 7, flags)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("name", ImGuiTableColumnFlags_WidthStretch, 1.5f);
        ImGui::TableSetupColumn("control", ImGuiTableColumnFlags_WidthStretch, 1.3f);
        ImGui::TableSetupColumn("alt m", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("agl m", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("tas", ImGuiTableColumnFlags_WidthStretch, 0.9f);
        ImGui::TableSetupColumn("hdg", ImGuiTableColumnFlags_WidthStretch, 0.8f);
        ImGui::TableSetupColumn("state", ImGuiTableColumnFlags_WidthStretch, 1.5f);
        ImGui::TableHeadersRow();
        for (std::size_t i = 0; i < batch_->states.size(); ++i) {
            const auto* meta = i < vehicles_.size() ? &vehicles_[i] : nullptr;
            if (meta && !meta->alive) continue;
            const auto& s = batch_->states[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            char label[96];
            if (meta) std::snprintf(label, sizeof label, "%s##%zu", meta->name.c_str(), i);
            else std::snprintf(label, sizeof label, "v%zu", i);
            if (ImGui::Selectable(label, static_cast<int>(i) == selected, ImGuiSelectableFlags_SpanAllColumns))
                controls_->selectedVehicle.store(static_cast<int>(i), std::memory_order_relaxed);
            ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(meta ? meta->level : "");
            if (s.diverged) {
                for (int c = 2; c <= 5; ++c) { ImGui::TableSetColumnIndex(c); ImGui::TextUnformatted("-"); }
                ImGui::TableSetColumnIndex(6);
                ImGui::TextUnformatted("DIVERGED");
                continue;
            }
            ImGui::TableSetColumnIndex(2); ImGui::Text("%.0f", s.altitudeMslM);
            ImGui::TableSetColumnIndex(3); ImGui::Text("%.0f", s.altitudeAglM);
            ImGui::TableSetColumnIndex(4); ImGui::Text("%.0f", s.airspeedTrueMs);
            ImGui::TableSetColumnIndex(5); ImGui::Text("%.0f", units::radiansToDegrees(s.eulerRad[2]));
            ImGui::TableSetColumnIndex(6);
            // (its abnormal state, where its Vehicle Interface says one: the gravest)
            const char* word = "";
            const int level = status_ && i < status_->size() && !s.onGround ? severity((*status_)[i], &word) : 0;
            if (level > 0) ImGui::TextColored(level >= 2 ? warningText() : cautionText(), "%s", word);
            else ImGui::TextUnformatted(s.onGround ? "ground" : "airborne");
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

} // namespace fsim::ui
