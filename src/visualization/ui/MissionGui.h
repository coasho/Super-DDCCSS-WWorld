#pragma once

// Scheme 3 Extensible Plugin Dock & Mission Systems Visualizers (ADR-30, MS-4, VIS-M14):
// Hosts all context-aware, aircraft-bound sensor & physics plugin panels along an
// elastic right-edge dock governed by ViewerControls::workspacePreset:
//   - kPanelRadar  : Airborne Radar B-Scope / PPI (mandatory on radar-equipped aircraft; hidden on missiles/helis/GA)
//   - kPanelSignal : Visual Radar Signal Lab (Range-Doppler heatmap + CFAR, Burn-Through A-Scope SNR curve, Polar Pattern)
//   - kPanelEw     : EMSO / EW RF Spectrum & ESM Bearing / CRLB Geolocation
//   - kPanelWeapon : 5-D Weapon DLZ Range Bar, LCOS / CCIP & Mounted Stores
//   - kPanelAero   : 6-DOF Aerodynamics, Flight Controls & up to 8-Engine / Rotor Propulsion
//   - kPanelComms  : Link 16 / C2 / Mode 5 IFF / PNT Drift & Native Multi-Sensor Fusion

#include "contracts/world_stream/FusionFrameV1.h"
#include "contracts/world_stream/TacticalOverlayLayoutV1.h"
#include "contracts/world_stream/WorldLayout.h"
#include "foundation/transport/TacticalOverlaySegmentV1.h"
#include "fsim/VehicleState.h"
#include "visualization/ui/ViewerControls.h"

#include <vsg/all.h>

#include <memory>
#include <string>
#include <vector>

namespace fsim::ipc {
class WorldMirror;
}

namespace fsim::ui {

class MissionGui : public vsg::Inherit<vsg::Command, MissionGui> {
public:
    explicit MissionGui(std::shared_ptr<ViewerControls> controls);

    /// Live mirror update: read followed vehicle's mission record, fusion table,
    /// vehicle state, and companion tactical telemetry frame.
    void update(const ipc::WorldMirror* mirror,
                int slot,
                const std::string& name,
                double simTime,
                const ipc::TacticalOverlayFrameV1* tactical = nullptr,
                const sim::VehicleState* ownState = nullptr,
                const std::string& type = {});

    /// Replay mode update: bind Radar & all workspace plugin panels strictly to the
    /// focused vehicle using recorded VehicleState + TacticalOverlayFrameV1.
    void updateReplay(const ipc::TacticalOverlayFrameV1* tactical,
                      const sim::VehicleState* ownState,
                      int slot,
                      const std::string& name,
                      const std::string& type,
                      double simTime);

    /// Whether a vehicle type string represents an aircraft carrying a cockpit radar display.
    static bool supportsAirborneRadar(const std::string& aircraftType) noexcept;

    void record(vsg::CommandBuffer&) const override;

private:
    struct DockRect {
        float x = 0.0f, y = 0.0f, w = 360.0f, h = 240.0f;
    };

    void drawRadarPanel(const DockRect& rect) const;
    void drawSignalLabPanel(const DockRect& rect) const;
    void drawEwPanel(const DockRect& rect) const;
    void drawWeaponPanel(const DockRect& rect) const;
    void drawAeroPanel(const DockRect& rect) const;
    void drawCommsPanel(const DockRect& rect) const;

    void drawLiveRadarBody(const ipc::MissionRadar& radar) const;
    void drawReplayRadarBody(const ipc::TacticalRadarVolumeWireV1& radar) const;
    void drawStandbyRadarBody() const;
    void drawNativeRangeDoppler(const ipc::MissionRangeDoppler& map) const;
    void drawDerivedRangeDoppler(const ipc::TacticalRadarVolumeWireV1* tact, const ipc::MissionRadar* live) const;
    void drawBurnThroughCurve(const ipc::TacticalRadarVolumeWireV1* tact, const ipc::MissionRadar* live) const;
    void drawAntennaPolar(const ipc::TacticalRadarVolumeWireV1* tact, const ipc::MissionRadar* live) const;

    void cacheFocusedTactical(const ipc::TacticalOverlayFrameV1* tactical, int slot);

    std::shared_ptr<ViewerControls> controls_;
    std::unique_ptr<ipc::MissionRecord> record_;
    std::unique_ptr<ipc::FusionRecordV1> fusion_;
    bool fusionValid_ = false;
    bool valid_ = false;
    bool mirrored_ = false;
    bool replayMode_ = false;
    bool supportsRadar_ = false;

    bool hasOwnState_ = false;
    sim::VehicleState ownState_{};
    double prevTargetRangeM_ = -1.0;
    double prevTargetRangeTimeS_ = -1.0;
    double estClosingMs_ = 0.0;

    // Focused-vehicle slices cached from TacticalOverlayFrameV1 (both live & replay)
    std::vector<ipc::TacticalRadarVolumeWireV1> focusedRadars_;
    std::vector<ipc::TacticalJammerLobeWireV1> relevantJammers_;
    std::vector<ipc::TacticalEsmRayWireV1> focusedEsm_;
    std::vector<ipc::TacticalWeaponCueWireV1> focusedWeapons_;
    std::vector<ipc::TacticalLinkPulseWireV1> focusedLinks_;
    std::vector<ipc::TacticalPropulsionExtWireV1> focusedProp_;

    int slot_ = -1;
    std::string name_;
    std::string type_;
    double simTime_ = 0.0;
};

} // namespace fsim::ui
