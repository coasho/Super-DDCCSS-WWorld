#pragma once

// Mission Systems windows (ADR-30, MS-4): the payloads of the vehicle the
// camera follows - their states, warm-ups and built-in tests, the capabilities
// they give - and each radar's display: a B-scope (azimuth across, range up)
// for a fighter's radar, a PPI (heading up) for a rotodome; for a radar at the
// signal level (MS-6), its range-Doppler map too. Everything drawn
// comes from the world's shared memory (ipc::MissionRecord): the payloads the
// SDK created and commands, never a simulation of the viewer's own. Each
// window collapses from its title bar; key p shows or hides them all.

#include "ipc/WorldLayout.h"
#include "ui/ViewerControls.h"

#include <vsg/all.h>

#include <memory>
#include <string>

namespace fsim::ipc {
class WorldMirror;
}

namespace fsim::ui {

class MissionGui : public vsg::Inherit<vsg::Command, MissionGui> {
public:
    explicit MissionGui(std::shared_ptr<ViewerControls> controls);

    /// Once per frame: the record of the vehicle in `slot` (named `name`) from `mirror` if it changed; null mirror, or
    /// slot < 0, when there is none (the viewer's own demo, or waiting for a world).
    void update(const ipc::WorldMirror* mirror, int slot, const std::string& name, double simTime);

    void record(vsg::CommandBuffer&) const override;

private:
    void drawPayloads() const;
    void drawRadar(const ipc::MissionRadar& radar) const;
    void drawRangeDoppler(const ipc::MissionRangeDoppler& map) const;

    std::shared_ptr<ViewerControls> controls_;
    std::unique_ptr<ipc::MissionRecord> record_;
    bool valid_ = false;     ///< a record read for the slot shown
    bool mirrored_ = false;  ///< a world is attached (else nothing to show)
    mutable bool showHits_ = true; ///< the radar displays' raw detections (else their tracks alone)
    int slot_ = -1;
    std::string name_;
    double simTime_ = 0.0;
};

} // namespace fsim::ui
