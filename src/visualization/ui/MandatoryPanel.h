#pragma once

#include <vsgImGui/imgui.h>

namespace fsim::ui {

/// Every populated Viewer panel stays open, including after loading an old collapsed layout.
/// Pair with ImGui::End(), even when Begin returns false because the window is clipped.
inline bool beginMandatoryPanel(const char* name, ImGuiWindowFlags flags = 0) {
    ImGui::SetNextWindowCollapsed(false, ImGuiCond_Always);
    return ImGui::Begin(name, nullptr, flags | ImGuiWindowFlags_NoCollapse);
}

} // namespace fsim::ui
