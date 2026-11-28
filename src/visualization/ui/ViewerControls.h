#pragma once

#include <atomic>
#include <cstdint>

namespace fsim::ui {

// VIS-M14 toggleable 3-D Tactical & EMSO overlay bitmask (0 = OFF, 0.0 ms cost).
inline constexpr std::uint32_t kOverlayRadar   = 1u << 0; ///< Radar scan wedges, active beams, STT lock, RCS envelopes
inline constexpr std::uint32_t kOverlayJammer  = 1u << 1; ///< EA jamming cones & live radar burn-through pockets
inline constexpr std::uint32_t kOverlayEsm     = 1u << 2; ///< ESM/RWR DF rays & 3-D Gauss-Newton FIM CRLB ellipsoids
inline constexpr std::uint32_t kOverlayLinks   = 1u << 3; ///< Link 16/DMS pulses, C2 consent, Mode 5 IFF, PNT GNSS drift
inline constexpr std::uint32_t kOverlayWeapons = 1u << 4; ///< 5-D launchZoneV2 DLZ/WEZ, Gun LCOS funnel, Mk 82 CCIP pipper
inline constexpr std::uint32_t kOverlaySwaths  = 1u << 5; ///< SAR spotlight footprints & Weather radar reflectivity swaths
inline constexpr std::uint32_t kOverlayAll     = 0x3Fu;

// Scheme 3 Role-Based Workspace Presets & Extensible Plugin Panel Masks.
enum class WorkspacePreset : int {
    Clean = 0,    ///< Pure spectator: top bar + mandatory Radar (if supported by focused aircraft)
    AirRadar = 1, ///< Air combat radar: Radar + Visual Signal Lab (R-D/SNR/Pattern) + Fleet
    Emso = 2,     ///< EMSO / EW: Radar + Visual Signal Lab + EW/ESM Spectrum + Fleet
    Weapon = 3,   ///< Weapons & guidance: DLZ/Weapon + 6-DOF Aero/Propulsion + Fleet
    Flight = 4,   ///< Flight dynamics: 6-DOF Aero/Propulsion + Comms/Fusion + Fleet
    Custom = 5,   ///< User-selected custom plugin combination
};
inline constexpr int kWorkspacePresetCount = 6;

inline constexpr std::uint32_t kPanelRadar   = 1u << 0; ///< Airborne Radar B-Scope / PPI (mandatory on radar aircraft)
inline constexpr std::uint32_t kPanelSignal  = 1u << 1; ///< Radar Signal Lab: Range-Doppler / Burn-Through SNR / Polar Pattern
inline constexpr std::uint32_t kPanelEw      = 1u << 2; ///< EMSO / EW Spectrum & ESM Bearing / CRLB panel
inline constexpr std::uint32_t kPanelWeapon  = 1u << 3; ///< 5-D Weapon DLZ / LCOS / CCIP & Ordnance panel
inline constexpr std::uint32_t kPanelAero    = 1u << 4; ///< 6-DOF Aerodynamics, Flight Controls & 8-Engine / Rotor Propulsion
inline constexpr std::uint32_t kPanelComms   = 1u << 5; ///< Link 16 / C2 / Mode 5 IFF / PNT & Native Fusion panel
inline constexpr std::uint32_t kPanelFleet   = 1u << 6; ///< Bottom-left Fleet roster & focus selector
inline constexpr std::uint32_t kPanelAll     = 0x7Fu;

/// Resolve the active plugin panel bitmask for a workspace preset.
inline constexpr std::uint32_t panelsForWorkspace(WorkspacePreset preset, std::uint32_t customMask) noexcept {
    switch (preset) {
    case WorkspacePreset::Clean:    return kPanelRadar;
    case WorkspacePreset::AirRadar: return kPanelRadar | kPanelSignal | kPanelFleet;
    case WorkspacePreset::Emso:     return kPanelRadar | kPanelSignal | kPanelEw | kPanelFleet;
    case WorkspacePreset::Weapon:   return kPanelRadar | kPanelWeapon | kPanelAero | kPanelFleet;
    case WorkspacePreset::Flight:   return kPanelRadar | kPanelAero | kPanelComms | kPanelFleet;
    case WorkspacePreset::Custom:   return customMask | kPanelRadar;
    }
    return kPanelRadar | kPanelSignal | kPanelFleet;
}

/// State shared between the ImGui layers, the key handler and the viewer loop
/// (all on the render thread) and the sim runner (its own thread). Plain
/// atomics: UI actions never take a lock the simulation could hold (design 6.3).
struct ViewerControls {
    std::atomic<bool> paused{false};
    std::atomic<bool> singleStep{false};
    std::atomic<double> seekTo{-1.0};   ///< replay: jump to this simulation time (< 0 = none), consumed by the loop
    // Replay extent written by the loop for the GUI (both 0 when not replaying).
    std::atomic<double> replayStart{0.0}, replayEnd{0.0};
    std::atomic<int> selectedVehicle{0};
    std::atomic<int> attachWorld{-1};    ///< mirror: index into Source::available to attach to (consumed by the loop)
    std::atomic<int> selectStep{0};      ///< +1 / -1: move the selection to the next / previous live vehicle
    std::atomic<int> cameraMode{0};      ///< world::CameraController::Mode
    std::atomic<double> cameraZoom{1.0}; ///< multiplicative request, consumed each frame
    std::atomic<bool> cameraReset{false};
    // VIS-M14 Tactical/EMSO 3-D overlay controls (toggled via MonitorGui or --overlays).
    std::atomic<std::uint32_t> tacticalOverlayMask{0u}; ///< bitmask of kOverlay* (default 0 = OFF)
    // Scheme 3 extensible workspace & plugin panel controls.
    std::atomic<int> workspacePreset{static_cast<int>(WorkspacePreset::AirRadar)};
    std::atomic<std::uint32_t> customPanelMask{kPanelRadar | kPanelSignal | kPanelFleet};
    std::atomic<int> radarSignalTab{0}; ///< 0=Range-Doppler, 1=Burn-Through A-Scope, 2=Antenna Polar Pattern
    // Live UI dock extents written by MonitorGui / MissionGui so 3-D zone annotations avoid active panels exactly.
    std::atomic<float> topBarBottomPx{0.0f};
    std::atomic<float> topBarRightPx{0.0f};
    std::atomic<float> rightDockLeftPx{0.0f};
    std::atomic<float> leftDockRightPx{0.0f};
    std::atomic<float> leftDockTopPx{0.0f};
    // Camera telemetry written by the loop for the GUI.
    std::atomic<double> eyeLatDeg{0.0}, eyeLonDeg{0.0}, eyeAltM{0.0}, eyeDistanceM{0.0};
    std::atomic<bool> quit{false};

    // Read-only telemetry written by the loop for the GUI.
    std::atomic<double> fps{0.0};
    std::atomic<double> frameMs{0.0};
    std::atomic<double> simThroughput{0.0}; ///< vehicle-steps/s
    std::atomic<double> simTime{0.0};
    std::atomic<std::uint64_t> snapshotSequence{0};
};

} // namespace fsim::ui
