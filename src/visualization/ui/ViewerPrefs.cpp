#include "visualization/ui/ViewerPrefs.h"

#include "foundation/core/Json.h"
#include "foundation/core/Log.h"
#include "visualization/scene/CameraController.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>

namespace fsim::ui {

namespace {

bool nearlyEqual(double a, double b, double eps) noexcept {
    return std::abs(a - b) <= eps;
}

} // namespace

bool ViewerPrefs::operator==(const ViewerPrefs& o) const noexcept {
    return cameraMode == o.cameraMode &&
           tacticalOverlayMask == o.tacticalOverlayMask &&
           clearCloudsAboveEntities == o.clearCloudsAboveEntities &&
           workspacePreset == o.workspacePreset &&
           customPanelMask == o.customPanelMask &&
           nearlyEqual(panelScale, o.panelScale, 1e-3) &&
           nearlyEqual(rightDockWidthPx, o.rightDockWidthPx, 0.25) &&
           nearlyEqual(solarLocalHours, o.solarLocalHours, 1e-3);
}

ViewerPrefs sanitizeViewerPrefs(ViewerPrefs p) noexcept {
    p.cameraMode = std::clamp(p.cameraMode, 0, world::CameraController::kModeCount - 1);
    p.tacticalOverlayMask &= kOverlayAll;
    p.workspacePreset = std::clamp(p.workspacePreset, 0, kWorkspacePresetCount - 1);
    p.customPanelMask &= kPanelAll;
    p.panelScale = std::isfinite(p.panelScale) ? std::clamp(p.panelScale, 0.75f, 1.35f) : 1.0f;
    p.rightDockWidthPx = std::isfinite(p.rightDockWidthPx) ? std::clamp(p.rightDockWidthPx, 280.0f, 520.0f) : 370.0f;
    if (!std::isfinite(p.solarLocalHours)) {
        p.solarLocalHours = 13.5;
    } else if (p.solarLocalHours < 0.0) {
        p.solarLocalHours = -1.0; // real-time UTC
    } else {
        p.solarLocalHours = std::fmod(p.solarLocalHours, 24.0);
        if (p.solarLocalHours < 0.0) p.solarLocalHours += 24.0;
    }
    return p;
}

ViewerPrefs captureViewerPrefs(const ViewerControls& controls, double fallbackSolarHours) noexcept {
    ViewerPrefs p;
    p.cameraMode = controls.cameraMode.load(std::memory_order_relaxed);
    p.tacticalOverlayMask = controls.tacticalOverlayMask.load(std::memory_order_relaxed);
    p.clearCloudsAboveEntities = controls.clearCloudsAboveEntities.load(std::memory_order_relaxed);
    p.workspacePreset = controls.workspacePreset.load(std::memory_order_relaxed);
    p.customPanelMask = controls.customPanelMask.load(std::memory_order_relaxed);
    p.panelScale = controls.panelScale.load(std::memory_order_relaxed);
    p.rightDockWidthPx = controls.rightDockWidthPx.load(std::memory_order_relaxed);
    const double rawSolar = controls.solarLocalHours.load(std::memory_order_relaxed);
    p.solarLocalHours = (rawSolar <= -1.5) ? fallbackSolarHours : rawSolar;
    return sanitizeViewerPrefs(p);
}

void applyViewerPrefs(const ViewerPrefs& prefs, ViewerControls& controls) noexcept {
    const ViewerPrefs s = sanitizeViewerPrefs(prefs);
    controls.cameraMode.store(s.cameraMode, std::memory_order_relaxed);
    controls.tacticalOverlayMask.store(s.tacticalOverlayMask, std::memory_order_relaxed);
    controls.clearCloudsAboveEntities.store(s.clearCloudsAboveEntities, std::memory_order_relaxed);
    controls.workspacePreset.store(s.workspacePreset, std::memory_order_relaxed);
    controls.customPanelMask.store(s.customPanelMask, std::memory_order_relaxed);
    controls.panelScale.store(s.panelScale, std::memory_order_relaxed);
    controls.rightDockWidthPx.store(s.rightDockWidthPx, std::memory_order_relaxed);
    controls.solarLocalHours.store(s.solarLocalHours, std::memory_order_relaxed);
}

bool loadViewerPrefs(const std::filesystem::path& path, ViewerPrefs& out) noexcept {
    try {
        std::error_code ec;
        if (path.empty() || !std::filesystem::exists(path, ec)) return false;
        std::ifstream in(path, std::ios::binary);
        if (!in) return false;
        const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
        const core::Json doc = core::Json::parse(text, path.string());
        if (!doc.isObject()) return false;

        ViewerPrefs p = out;
        if (doc.has("cameraMode")) p.cameraMode = static_cast<int>(std::llround(doc.number("cameraMode", p.cameraMode)));
        if (doc.has("tacticalOverlayMask")) {
            const double v = doc.number("tacticalOverlayMask", static_cast<double>(p.tacticalOverlayMask));
            if (std::isfinite(v) && v >= 0.0) p.tacticalOverlayMask = static_cast<std::uint32_t>(std::llround(v));
        }
        if (doc.has("clearCloudsAboveEntities")) {
            p.clearCloudsAboveEntities = doc.boolean("clearCloudsAboveEntities", p.clearCloudsAboveEntities);
        }
        if (doc.has("workspacePreset")) {
            p.workspacePreset = static_cast<int>(std::llround(doc.number("workspacePreset", p.workspacePreset)));
        }
        if (doc.has("customPanelMask")) {
            const double v = doc.number("customPanelMask", static_cast<double>(p.customPanelMask));
            if (std::isfinite(v) && v >= 0.0) p.customPanelMask = static_cast<std::uint32_t>(std::llround(v));
        }
        if (doc.has("panelScale")) p.panelScale = static_cast<float>(doc.number("panelScale", p.panelScale));
        if (doc.has("rightDockWidthPx")) {
            p.rightDockWidthPx = static_cast<float>(doc.number("rightDockWidthPx", p.rightDockWidthPx));
        }
        if (doc.has("solarLocalHours")) p.solarLocalHours = doc.number("solarLocalHours", p.solarLocalHours);

        out = sanitizeViewerPrefs(p);
        return true;
    } catch (const std::exception& e) {
        LOG_WARN("ui") << "ignoring viewer prefs " << path.string() << ": " << e.what();
        return false;
    } catch (...) {
        return false;
    }
}

bool saveViewerPrefs(const std::filesystem::path& path, const ViewerPrefs& prefs) noexcept {
    try {
        if (path.empty()) return false;
        const ViewerPrefs s = sanitizeViewerPrefs(prefs);
        std::error_code ec;
        if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path(), ec);

        char buf[512];
        const int n = std::snprintf(
            buf, sizeof buf,
            "{\n"
            "  \"version\": 1,\n"
            "  \"cameraMode\": %d,\n"
            "  \"tacticalOverlayMask\": %u,\n"
            "  \"clearCloudsAboveEntities\": %s,\n"
            "  \"workspacePreset\": %d,\n"
            "  \"customPanelMask\": %u,\n"
            "  \"panelScale\": %.3f,\n"
            "  \"rightDockWidthPx\": %.1f,\n"
            "  \"solarLocalHours\": %.3f\n"
            "}\n",
            s.cameraMode,
            static_cast<unsigned>(s.tacticalOverlayMask),
            s.clearCloudsAboveEntities ? "true" : "false",
            s.workspacePreset,
            static_cast<unsigned>(s.customPanelMask),
            static_cast<double>(s.panelScale),
            static_cast<double>(s.rightDockWidthPx),
            s.solarLocalHours);
        if (n <= 0 || static_cast<std::size_t>(n) >= sizeof buf) return false;

        std::filesystem::path tmp = path;
        tmp += ".tmp";
        {
            std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
            if (!out) return false;
            out.write(buf, n);
            out.flush();
            if (!out.good()) return false;
        }
        std::filesystem::rename(tmp, path, ec);
        if (ec) {
            // On Windows, rename fails if target exists in some std::filesystem implementations; replace explicitly.
            std::error_code ecRm;
            std::filesystem::remove(path, ecRm);
            ec.clear();
            std::filesystem::rename(tmp, path, ec);
            if (ec) {
                std::filesystem::remove(tmp, ecRm);
                return false;
            }
        }
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace fsim::ui
