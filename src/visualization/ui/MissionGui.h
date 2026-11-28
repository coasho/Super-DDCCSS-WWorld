#pragma once

// Mission Systems windows (ADR-30, MS-4): the payloads and radar displays of
// the vehicle the camera follows. In live mirror mode everything comes from the
// world's shared memory (ipc::MissionRecord); in replay mode radar-equipped
// aircraft automatically bind their native B-Scope / PPI display to recorded
// TacticalOverlayFrameV1 telemetry (or standby state for supported radar types),
// while vehicles without airborne radar (missiles, utility helicopters, GA)
// automatically keep the Radar UI hidden.

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

    /// Live mirror update: read the followed vehicle's mission record & fusion table.
    void update(const ipc::WorldMirror* mirror, int slot, const std::string& name, double simTime);

    /// Replay mode update: bind Radar UI strictly to the focused vehicle when it carries
    /// a radar (live TacticalRadarVolumeWireV1 or radar-equipped airframe in standby), and
    /// hide Radar UI when focusing missiles / non-radar vehicles.
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
    void drawPayloads() const;
    void drawRadar(const ipc::MissionRadar& radar) const;
    void drawRangeDoppler(const ipc::MissionRangeDoppler& map) const;
    void drawReplayRadar(const ipc::TacticalRadarVolumeWireV1& radar, std::uint32_t index) const;
    void drawReplayStandbyRadar() const;

    std::shared_ptr<ViewerControls> controls_;
    std::unique_ptr<ipc::MissionRecord> record_;
    std::unique_ptr<ipc::FusionRecordV1> fusion_;
    bool fusionValid_ = false;
    bool valid_ = false;
    bool mirrored_ = false;

    // Replay-mode radar binding state (focused vehicle only)
    bool replayMode_ = false;
    bool replaySupportsRadar_ = false;
    bool hasOwnState_ = false;
    sim::VehicleState ownState_{};
    std::vector<ipc::TacticalRadarVolumeWireV1> replayRadars_;

    int slot_ = -1;
    std::string name_;
    std::string type_;
    double simTime_ = 0.0;
};

} // namespace fsim::ui
