#pragma once

// Colours for the ImGui layers, given as sRGB - as everyone picks them - and made what the render target takes.

#include <vsgImGui/imgui.h>

#include <algorithm>
#include <cmath>

namespace fsim::ui {

/// Effective UI layout scale factor: stays consistent whether the viewer rasterized a native
/// high-DPI TrueType font (FontGlobalScale == 1, FontSize ~ 21*dpi) or a headless test scaled
/// the default 13 px bitmap font via FontGlobalScale.
inline float uiScale() noexcept {
    const float byFont = ImGui::GetFontSize() / 16.0f;
    const float byGlobal = ImGui::GetIO().FontGlobalScale;
    return std::max({byFont, byGlobal, 0.75f});
}

/// Whether the ImGui renderer writes to an sRGB target, its style's colours made linear for it (the dark style's window
/// background, 0.06, is then some 0.005): colours given here are sRGB, and made linear likewise.
inline bool linearTarget() { return ImGui::GetStyle().Colors[ImGuiCol_WindowBg].x < 0.03f; }

inline float toLinear(float c) { return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f); }

inline ImVec4 tint(float r, float g, float b, float a = 1.0f) {
    if (!linearTarget()) return ImVec4(r, g, b, a);
    return ImVec4(toLinear(r), toLinear(g), toLinear(b), a);
}

inline ImU32 colour(int r, int g, int b, int a = 255) {
    return ImGui::ColorConvertFloat4ToU32(tint(static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f, static_cast<float>(b) / 255.0f,
                                               static_cast<float>(a) / 255.0f));
}

/// A caution's amber and a warning's red, as text: a state that wants watching, and one that wants acting on.
inline ImVec4 cautionText() { return tint(1.0f, 0.78f, 0.25f); }
inline ImVec4 warningText() { return tint(1.0f, 0.38f, 0.32f); }

} // namespace fsim::ui
