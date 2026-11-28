#pragma once

#include "contracts/world_stream/WorldLayout.h"
#include "contracts/world_stream/SnapshotBatch.h"
#include "visualization/ui/ViewerControls.h"

#include <vsg/all.h>

#include <memory>
#include <string>
#include <vector>

namespace fsim::ui {

/// ImGui monitor + vehicle list layers (design 9.5). A vsg::Command recorded
/// by vsgImGui::RenderImGui each frame; it reads the latest snapshot batch the
/// viewer loop has set and edits ViewerControls.
class MonitorGui : public vsg::Inherit<vsg::Command, MonitorGui> {
public:
    MonitorGui(std::shared_ptr<ViewerControls> controls, std::string aircraft);

    /// Called by the viewer loop before recording each frame.
    void setBatch(const sim::SnapshotBatch* batch) { batch_ = batch; }

    /// Where the vehicles come from: the built-in demo or a mirrored world.
    struct Source {
        bool mirror = false;
        bool attached = false;
        std::string world;              ///< world name (mirror) or aircraft (demo)
        double ageSeconds = 0.0;        ///< since the last publish
        bool publisherAlive = false;
        double simTime = 0.0;
        double vehicleStepsPerSecond = 0.0;
        std::string environment;        ///< one-line summary
        std::vector<std::string> available; ///< worlds seen in the registry while waiting
    };
    void setSource(Source source) { source_ = std::move(source); }

    /// Per-slot identity (mirror mode); empty = "v<i>" for every slot.
    struct VehicleMeta {
        std::string name, type;
        bool alive = true;
        const char* level = "";
    };
    void setVehicles(std::vector<VehicleMeta> vehicles) { vehicles_ = std::move(vehicles); }
    const std::vector<VehicleMeta>& vehicles() const noexcept { return vehicles_; }

    /// Each slot's flight status as the world's Vehicle Interface has it (mirror mode: the mirror's, read with the
    /// batch; null: none known - the demo, a replay). docs/vehicle-interface.md, 7.3.
    void setStatus(const std::vector<ipc::FlightStatus>* status) { status_ = status; }
    /// What a status calls for: 0 nothing, 1 a caution (low on energy: flown on its speed floor), 2 a warning (it
    /// deviates from the altitude it commands; a wing upset, far past its angle-of-attack limit). `word`, where given:
    /// what to say of it (the gravest), empty for 0.
    static int severity(const ipc::FlightStatus& status, const char** word = nullptr) noexcept;

    /// Screen-space known-entity / radar-track labels for this frame (window pixels).
    /// When `offscreen` is true, MonitorGui pins a sleek directional indicator at the safe viewport edge.
    struct Label {
        float x = 0.0f, y = 0.0f;
        std::string text;
        bool selected = false;
        int severity = 0;          ///< severity() of own vehicle: amber (1) or red (2)
        std::uint8_t iconKind = 0; ///< 0=neutral, 1=blue solid diamond, 2=red hollow diamond, 3=weapon triangle, 4=radar contact/track
        bool offscreen = false;    ///< outside camera frustum or behind eye -> render edge directional badge
        float dirX = 0.0f, dirY = 0.0f; ///< unit screen direction from viewport center toward off-screen entity
        float rangeKm = -1.0f;     ///< slant range in km (shown on edge indicator when >= 0)
    };
    /// Project a 3-D ECEF position into a Label's on-screen coordinates or off-screen edge direction.
    static void projectLabel(const vsg::dvec3& posEcef, const vsg::dmat4& view, const vsg::dmat4& proj,
                             std::uint32_t width, std::uint32_t height, Label& out) noexcept;
    void setLabels(std::vector<Label> labels) { labels_ = std::move(labels); }

    void record(vsg::CommandBuffer&) const override;

private:
    void drawMonitor() const;
    void drawReplayBar() const;
    void drawVehicleList() const;
    /// A vehicle's flight status, as items along a line; `state`: its abnormal state's word first.
    void drawStatus(const sim::VehicleState& s, const ipc::FlightStatus& status, bool state) const;

    void drawLabels() const;

    std::shared_ptr<ViewerControls> controls_;
    std::string aircraft_;
    const sim::SnapshotBatch* batch_ = nullptr;
    std::vector<Label> labels_;
    Source source_;
    std::vector<VehicleMeta> vehicles_;
    const std::vector<ipc::FlightStatus>* status_ = nullptr;
};

} // namespace fsim::ui
