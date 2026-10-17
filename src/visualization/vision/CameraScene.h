#pragma once

// Vehicle cameras' scene (ARCH-06; design 8.4): the Earth, sky and vehicle models drawn offscreen for cameras mounted
// on vehicles. It renders read-only frames - each vehicle's state, model and what it carries, as a source reads them -
// and never reads or steps a World: the compatible World-handle API (src/adapters/vision) gathers the frames.

#include "visualization/vision/Offscreen.h"

#include "fsim/VehicleState.h"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fsim::world {
class SkyDome;
class VehicleVisuals;
}

namespace fsim::vision {

/// How the scene is built: the rendering part of the public vision::Options.
struct CameraSceneSettings {
    bool debugLayer = false;
    bool sky = true;
    bool earth = true;
    bool imagery = true;
    std::string imageryUrl;
    unsigned maxLevel = 17;
    bool elevation = true;
    std::string elevationUrl;
    std::string assetDir;
    bool segmentation = false;
    unsigned maxVehicles = 64;
};

/// One vehicle as a frame gives it.
struct VehicleFrame {
    std::uint32_t id = 0;
    std::string model, type;                      ///< what it is drawn as (taken when it first appears)
    sim::VehicleState state;                      ///< its state as the simulation reports it
    std::array<double, 3> cgFromModelOriginM{};   ///< its CG's place from its model's origin, body axes
    std::vector<std::pair<int, std::string>> carried; ///< each store on its station now: station, store
    std::vector<std::pair<int, double>> doors;    ///< each station's bay doors as they stand
};

/// A camera's mount on its vehicle.
struct CameraMount {
    std::array<double, 3> offsetBodyM{};
    double yawDeg = 0.0, pitchDeg = 0.0, rollDeg = 0.0;
    unsigned width = 1, height = 1;
    double fovDeg = 60.0;
    bool hideOwnVehicle = false;
    bool depth = false;
    bool segmentation = false;
};

/// The offscreen device could not be opened ("vision: ..."); the World-handle API reports it as its own error.
struct CameraSceneError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

class CameraScene {
public:
    /// Builds the scene; `utcSeconds` places the sun. Throws CameraSceneError when the offscreen device fails (other
    /// failures, the scene builders' own, propagate as they are).
    CameraScene(const CameraSceneSettings& settings, double utcSeconds);
    ~CameraScene();
    CameraScene(const CameraScene&) = delete;
    CameraScene& operator=(const CameraScene&) = delete;

    /// A camera on vehicle `vehicleId`; its index, the offscreen target's.
    unsigned addCamera(std::uint32_t vehicleId, const CameraMount& mount);
    std::size_t cameraCount() const noexcept;
    std::uint32_t cameraVehicle(std::size_t camera) const noexcept;

    /// The vehicles present now: new ones take a free slot (beyond maxVehicles, not drawn), absent ones free theirs.
    void update(const std::vector<VehicleFrame>& vehicles);
    /// Each camera's vehicle pose (indexed as the cameras; none: it keeps its last), then the sun from `utcSeconds`.
    void aim(const std::vector<std::optional<sim::VehicleState>>& cameraVehicles, double utcSeconds);

    /// A vehicle's segmentation id (0: not drawn).
    unsigned segmentationId(std::uint32_t vehicleId) const noexcept;

    Offscreen& offscreen() noexcept;
    const Offscreen& offscreen() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace fsim::vision
