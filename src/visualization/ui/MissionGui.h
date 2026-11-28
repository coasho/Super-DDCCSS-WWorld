#pragma once

// Extensible Right-Edge Plugin Dock & Mission Systems Visualizers (ADR-30, MS-4, VIS-M14):
// Strictly real native SDK / IPC data only — ZERO fabricated plots and ZERO omniscient truth peeking.
// Hosts optional, capability-gated own-platform panels governed by ViewerControls::workspacePreset:
//   - kPanelRadar  : Airborne Radar B-Scope / PPI (toggleable; hidden automatically on missiles/helis/GA)
//   - kPanelSignal : Authentic Native Signal-Level Range-Doppler (black CRT + green/yellow/white phosphor)
//   - kPanelEw     : Own-Platform EA Transmitter & Own RWR/ESM Intercepts (strictly own receiver data)
//   - kPanelWeapon : Own Fire-Control 5-D DLZ / LCOS / CCIP & Mounted Stores
//   - kPanelAero   : Own 6-DOF Aerodynamics, Flight Controls & up to 8-Engine / Rotor Propulsion
//   - kPanelComms  : Own Link 16 / C2 / Mode 5 IFF / PNT & Native Multi-Sensor Fusion

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

    /// Live mirror update: read followed vehicle's native mission record, fusion table,
    /// own VehicleState, and companion tactical telemetry frame.
    void update(const ipc::WorldMirror* mirror,
                int slot,
                const std::string& name,
                double simTime,
                const ipc::TacticalOverlayFrameV1* tactical = nullptr,
                const sim::VehicleState* ownState = nullptr,
                const std::string& type = {});

    /// Replay mode update: bind optional own-platform panels strictly to the
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
        float x = 0.0f, y = 0.0f, w = 370.0f, h = 240.0f;
        float fontScale = 1.0f;
    };

    void drawDismissButton(std::uint32_t panelBit, std::uint32_t activePanels) const;
    void drawRadarPanel(const DockRect& rect, std::uint32_t activePanels) const;
    void drawRangeDopplerPanel(const DockRect& rect, std::uint32_t activePanels) const;
    void drawEwPanel(const DockRect& rect, std::uint32_t activePanels) const;
    void drawWeaponPanel(const DockRect& rect, std::uint32_t activePanels) const;
    void drawAeroPanel(const DockRect& rect, std::uint32_t activePanels) const;
    void drawCommsPanel(const DockRect& rect, std::uint32_t activePanels) const;

    void drawLiveRadarBody(const ipc::MissionRadar& radar) const;
    void drawReplayRadarBody(const ipc::TacticalRadarVolumeWireV1& radar) const;
    void drawStandbyRadarBody() const;
    void drawNativeRangeDoppler(const ipc::MissionRangeDoppler& map) const;

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

    // Strictly own-platform slices cached from TacticalOverlayFrameV1 (zero omniscient enemy peeking!)
    std::vector<ipc::TacticalRadarVolumeWireV1> focusedRadars_;
    std::vector<ipc::TacticalJammerLobeWireV1> ownJammers_; ///< ONLY j.slot == slot_ (own EA transmitter)
    std::vector<ipc::TacticalEsmRayWireV1> focusedEsm_;      ///< ONLY e.observerSlot == slot_ (own RWR/ESM receiver)
    std::vector<ipc::TacticalWeaponCueWireV1> focusedWeapons_;
    std::vector<ipc::TacticalLinkPulseWireV1> focusedLinks_;
    std::vector<ipc::TacticalPropulsionExtWireV1> focusedProp_;

    int slot_ = -1;
    std::string name_;
    std::string type_;
    double simTime_ = 0.0;
};

} // namespace fsim::ui
