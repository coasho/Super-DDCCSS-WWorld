#pragma once

#include "visualization/ui/ViewerControls.h"

#include <cstdint>
#include <filesystem>

namespace fsim::ui {

/// Persistent user UI & scene preferences saved across flightsim-viewer sessions.
struct ViewerPrefs {
    int cameraMode = 1; ///< world::CameraController::Mode (default Orbit)
    std::uint32_t tacticalOverlayMask = 0u;
    bool clearCloudsAboveEntities = false;
    int workspacePreset = static_cast<int>(WorkspacePreset::AirRadar);
    std::uint32_t customPanelMask = kPanelRadar | kPanelSignal | kPanelFleet;
    float panelScale = 1.0f;
    float rightDockWidthPx = 370.0f;
    double solarLocalHours = 13.5; ///< >= 0.0 fixed local solar hour [0..24), -1.0 real-time UTC

    bool operator==(const ViewerPrefs& other) const noexcept;
    bool operator!=(const ViewerPrefs& other) const noexcept { return !(*this == other); }
};

/// Clamp every field in `prefs` to valid, safe UI ranges.
ViewerPrefs sanitizeViewerPrefs(ViewerPrefs prefs) noexcept;

/// Snapshot current interactive controls into a sanitized ViewerPrefs struct.
/// When controls.solarLocalHours is a CLI --sun-utc override (<= -1.5), preserves `fallbackSolarHours`.
ViewerPrefs captureViewerPrefs(const ViewerControls& controls, double fallbackSolarHours = 13.5) noexcept;

/// Push sanitized `prefs` into live ViewerControls atomics.
void applyViewerPrefs(const ViewerPrefs& prefs, ViewerControls& controls) noexcept;

/// Load persisted preferences from JSON file `path` into `out` (unspecified fields keep `out`'s defaults).
/// Returns true iff `path` existed and parsed cleanly.
bool loadViewerPrefs(const std::filesystem::path& path, ViewerPrefs& out) noexcept;

/// Atomically write `prefs` as formatted JSON to `path` (via temp file + rename).
/// Returns true on success.
bool saveViewerPrefs(const std::filesystem::path& path, const ViewerPrefs& prefs) noexcept;

} // namespace fsim::ui
