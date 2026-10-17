#include "visualization/vision/CameraScene.h"

#include "foundation/core/Log.h"
#include "foundation/io/AssetResolver.h"
#include "foundation/platform/Paths.h"
#include "visualization/scene/Earth.h"
#include "visualization/scene/Frames.h"
#include "visualization/scene/Sky.h"
#include "visualization/scene/SkyDome.h"
#include "visualization/scene/Terrain.h"
#include "visualization/scene/VehicleVisuals.h"

#include <chrono>
#include <filesystem>

namespace fsim::vision {

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;
}

struct CameraScene::Impl {
    CameraSceneSettings settings;
    Offscreen offscreen;
    vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid;
    std::unique_ptr<world::VehicleVisuals> visuals;
    std::unique_ptr<world::SkyDome> sky;
    vsg::ref_ptr<vsg::Node> sunLight;
    double lastSunUpdate = -1e9;
    vsg::dvec3 sunDir{1.0, 0.0, 0.0}; // toward the sun (ECEF), from the source's clock
    vsg::dvec3 lightPlace{0.0, 0.0, 0.0}; // where the scene is lit for: the first camera's eye

    struct Mount {
        std::uint32_t vehicleId = 0;
        vsg::dmat4 mount; ///< body -> camera frame (translation + attitude)
        vsg::dmat4 lastBodyToEcef;
        bool posed = false;
    };
    std::vector<Mount> mounts;
    std::vector<std::uint32_t> slotVehicle; ///< visuals slot -> vehicle id (0 = free)
    std::unordered_map<std::uint32_t, std::size_t> vehicleSlot;
    std::vector<sim::VehicleState> states;
    std::unordered_map<std::uint32_t, vsg::Mask> ownBit; ///< vehicles that hide themselves from their own cameras

    std::size_t slotFor(const VehicleFrame& v) {
        const auto it = vehicleSlot.find(v.id);
        if (it != vehicleSlot.end()) return it->second;
        for (std::size_t s = 0; s < slotVehicle.size(); ++s)
            if (slotVehicle[s] == 0) {
                slotVehicle[s] = v.id;
                vehicleSlot[v.id] = s;
                visuals->resetCarriage(s);
                visuals->setModel(s, v.model, v.type);
                // A cached model's segmentation copy may not have been drawn before.
                offscreen.sceneChanged();
                const auto bit = ownBit.find(v.id);
                visuals->setMask(s, bit != ownBit.end() ? bit->second : vsg::MASK_ALL);
                visuals->setVisible(s, true);
                return s;
            }
        return static_cast<std::size_t>(-1); // beyond maxVehicles: not drawn
    }
};

CameraScene::CameraScene(const CameraSceneSettings& settings, double utcSeconds) : impl_(std::make_unique<Impl>()) {
    impl_->settings = settings;
    Offscreen::Settings s;
    s.debugLayer = settings.debugLayer;
    std::string error;
    if (!impl_->offscreen.create(s, &error)) throw CameraSceneError("vision: " + error);

    auto vsgOptions = impl_->offscreen.options();
    impl_->ellipsoid = vsg::EllipsoidModel::create();
    auto scene = vsg::Group::create();

    int day = 172;
    double hours = 12.0;
    world::utcOf(utcSeconds, day, hours);
    const vsg::dvec3 sunDir = world::sunDirectionEcef(day, hours);
    if (settings.sky) {
        impl_->sky = std::make_unique<world::SkyDome>(vsgOptions, sunDir);
        scene->addChild(impl_->sky->node());
        impl_->sunLight = world::createSunLight(day, hours);
        scene->addChild(impl_->sunLight);
    } else {
        scene->addChild(vsg::createHeadlight());
    }

    world::EarthSettings earth;
    earth.source = settings.imagery ? (settings.imageryUrl.empty() ? world::EarthSettings::Source::EsriWorldImagery
                                                                   : world::EarthSettings::Source::Custom)
                                    : world::EarthSettings::Source::OpenStreetMap;
    if (!settings.imageryUrl.empty()) earth.imageryUrl = settings.imageryUrl;
    earth.maxLevel = settings.maxLevel;
    earth.elevationMaxLevel = std::min(15u, settings.maxLevel);
    if (settings.elevation) earth.elevationUrl = settings.elevationUrl.empty() ? world::kAwsTerrariumUrl : settings.elevationUrl;
    earth.offlineRoot = platform::mapsDir(); // (the local map set: layer URLs become its files, nothing is downloaded)
    if (!settings.earth) {
        // nothing: sky and vehicles only
    } else if (auto e = world::createEarth(earth, vsgOptions, impl_->ellipsoid)) {
        scene->addChild(e);
        if (auto caps = world::createPolarCaps(impl_->ellipsoid, vsgOptions)) scene->addChild(caps);
    } else {
        LOG_WARN("vision") << "no Earth in the camera scene";
    }

    io::AssetResolver assets;
    if (!settings.assetDir.empty()) assets.addSearchPath(settings.assetDir);
    world::VehicleVisuals::Settings vs;
    if (auto sample = assets.find("models/Cesium_Air.glb")) vs.modelPath = sample->string();
    world::VehicleVisuals::applyManifest(vs);
    for (const auto& dir : assets.searchPaths())
        if (std::filesystem::is_directory(dir / "models")) vs.modelDirs.push_back(dir / "models");
    for (const auto& dir : assets.aircraftDirs()) // aircraft of our own carry their model: <dir>/<type>/<type>.glb
        vs.modelDirs.push_back(dir);
    vs.segmentation = settings.segmentation;
    impl_->visuals = std::make_unique<world::VehicleVisuals>(settings.maxVehicles, vs, vsgOptions);
    for (unsigned i = 0; i < settings.maxVehicles; ++i) impl_->visuals->setVisible(i, false);
    scene->addChild(impl_->visuals->node());
    impl_->slotVehicle.assign(settings.maxVehicles, 0);
    impl_->states.assign(settings.maxVehicles, sim::VehicleState{});

    impl_->offscreen.setScene(scene, impl_->ellipsoid);
    if (settings.segmentation) impl_->offscreen.setSegmentationScene(impl_->visuals->segmentationNode());
    impl_->visuals->setCompiler([impl = impl_.get()](vsg::ref_ptr<vsg::Node>) {
        // update() attaches the complete model/store graph before render compiles it.
        impl->offscreen.sceneChanged();
        return true;
    });
}

CameraScene::~CameraScene() = default;

unsigned CameraScene::addCamera(std::uint32_t vehicleId, const CameraMount& spec) {
    Impl::Mount m;
    m.vehicleId = vehicleId;
    // Body -> camera: translate to the mount, then yaw (about z, down), pitch (about y, right), roll (about x, forward).
    m.mount = vsg::translate(spec.offsetBodyM[0], spec.offsetBodyM[1], spec.offsetBodyM[2]) *
              vsg::rotate(spec.yawDeg * kDeg, vsg::dvec3(0.0, 0.0, 1.0)) *
              vsg::rotate(spec.pitchDeg * kDeg, vsg::dvec3(0.0, 1.0, 0.0)) *
              vsg::rotate(spec.rollDeg * kDeg, vsg::dvec3(1.0, 0.0, 0.0));
    impl_->mounts.push_back(m);
    // Own-vehicle hiding: the vehicle's visuals carry one mask bit, its cameras' views clear that bit.
    vsg::Mask viewMask = vsg::MASK_ALL;
    if (spec.hideOwnVehicle) {
        auto it = impl_->ownBit.find(vehicleId);
        if (it == impl_->ownBit.end() && impl_->ownBit.size() < 31)
            it = impl_->ownBit.emplace(vehicleId, vsg::Mask(1u << (impl_->ownBit.size() + 1))).first;
        if (it != impl_->ownBit.end()) viewMask = vsg::MASK_ALL & ~it->second;
    }
    if (spec.segmentation && !impl_->settings.segmentation)
        LOG_WARN("vision") << "CameraSpec::segmentation needs Options::segmentation; this camera has no ids";
    return impl_->offscreen.addCamera(std::max(1u, spec.width), std::max(1u, spec.height), spec.fovDeg, viewMask, spec.depth,
                                      spec.segmentation);
}

std::size_t CameraScene::cameraCount() const noexcept { return impl_->mounts.size(); }

std::uint32_t CameraScene::cameraVehicle(std::size_t camera) const noexcept {
    return camera < impl_->mounts.size() ? impl_->mounts[camera].vehicleId : 0u;
}

void CameraScene::update(const std::vector<VehicleFrame>& vehicles) {
    auto& im = *impl_;
    std::vector<unsigned char> seen(im.slotVehicle.size(), 0);
    for (const auto& v : vehicles) {
        const std::size_t s = im.slotFor(v);
        if (s < seen.size()) {
            seen[s] = 1;
            // its picture hangs from its model's origin, its CG's place back from the position - as the viewer's
            // (ipc::originEcef): its stores are then on their stations where the simulation releases them
            im.states[s] = v.state;
            const std::array<double, 3>& cg = v.cgFromModelOriginM;
            const double* R = im.states[s].rotationBodyToEcef; // (row-major: its columns the body's axes)
            for (int r = 0; r < 3; ++r) im.states[s].positionEcef[r] -= R[r * 3 + 0] * cg[0] + R[r * 3 + 1] * cg[1] + R[r * 3 + 2] * cg[2];
            // and what it carries: each store on its station while it is on it, its bay's doors as they stand
            std::vector<world::VehicleVisuals::Carried> carried;
            for (const auto& [station, store] : v.carried)
                carried.push_back({station, world::VehicleVisuals::storeType(store)});
            im.visuals->setStores(s, carried);
            im.visuals->setBayDoors(s, v.doors);
        }
    }
    for (std::size_t s = 0; s < im.slotVehicle.size(); ++s)
        if (im.slotVehicle[s] != 0 && !seen[s]) {
            im.vehicleSlot.erase(im.slotVehicle[s]);
            im.slotVehicle[s] = 0;
            im.visuals->setVisible(s, false);
        }
    im.visuals->update(Span<const sim::VehicleState>(im.states));
}

void CameraScene::aim(const std::vector<std::optional<sim::VehicleState>>& cameraVehicles, double utcSeconds) {
    auto& im = *impl_;
    for (std::size_t i = 0; i < im.mounts.size(); ++i) {
        auto& m = im.mounts[i];
        if (i < cameraVehicles.size() && cameraVehicles[i]) {
            m.lastBodyToEcef = world::bodyToEcef(*cameraVehicles[i]);
            m.posed = true;
        }
        if (!m.posed || !im.offscreen.active(static_cast<unsigned>(i))) continue;
        const vsg::dmat4 cam = m.lastBodyToEcef * m.mount;
        const vsg::dvec3 eye = cam * vsg::dvec3(0.0, 0.0, 0.0);
        const vsg::dvec3 forward = cam * vsg::dvec3(1.0, 0.0, 0.0);
        const vsg::dvec3 above = cam * vsg::dvec3(0.0, 0.0, -1.0);
        im.offscreen.setView(static_cast<unsigned>(i), eye, forward, above - eye);
        if (i == 0 && im.sky) im.sky->update(eye);
        if (i == 0) im.lightPlace = eye; // (the lights are the scene's: lit for the first camera's place)
    }
    // Sun from the source's clock (once a second of wall time is plenty).
    if (im.sunLight) {
        const auto now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        if (now - im.lastSunUpdate > 1.0) {
            im.lastSunUpdate = now;
            int day = 172;
            double hours = 12.0;
            world::utcOf(utcSeconds, day, hours);
            im.sunDir = world::sunDirectionEcef(day, hours);
            if (im.sky) im.sky->setSun(im.sunDir);
        }
        if (vsg::length(im.lightPlace) > 1.0) world::setDaylight(im.sunLight, im.sunDir, im.lightPlace);
    }
}

unsigned CameraScene::segmentationId(std::uint32_t vehicleId) const noexcept {
    const auto it = impl_->vehicleSlot.find(vehicleId);
    return it == impl_->vehicleSlot.end() ? 0u : static_cast<unsigned>(it->second) + 1u;
}

Offscreen& CameraScene::offscreen() noexcept { return impl_->offscreen; }
const Offscreen& CameraScene::offscreen() const noexcept { return impl_->offscreen; }

} // namespace fsim::vision
