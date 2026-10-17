// fsim vision SDK: cameras on vehicles over the public World API (design 8.4). This is the compatible World-handle
// adapter (ARCH-06): it reads the World into read-only frames and shares images with viewers; vision::CameraScene
// (src/visualization/vision) renders them offscreen with the viewer's scene builders.

#include "fsim/Vision.h"

#include "foundation/transport/VisionSegment.h"
#include "visualization/vision/CameraScene.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC // vsgXchange carries its own copy
#define STBI_WRITE_NO_STDIO
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
#include <stb_image_write.h>
#pragma GCC diagnostic pop

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <unordered_map>

namespace fsim::vision {

struct Sensors::Impl {
    World* world = nullptr;
    Options options;
    std::unique_ptr<CameraScene> scene;
    double lastRenderMs = 0.0;
    bool compiled = false;

    // Sharing with viewers: the segment follows the active camera set.
    ipc::VisionPublisher publisher;
    std::vector<unsigned> publishedCameras; ///< active camera index per segment slot
    std::size_t publishedActive = 0;        ///< active-camera signature the table was built for
    std::uint64_t publishedTableVersion = 0;
    std::uint64_t cameraTableVersion = 0;   ///< bumped on add/remove

    void publish() {
        const Offscreen& offscreen = scene->offscreen();
        if (!options.publish || !world->published() || scene->cameraCount() == 0) return;
        if (!publisher.valid() && !publisher.create(world->name())) {
            options.publish = false; // once
            return;
        }
        if (publishedTableVersion != cameraTableVersion) {
            std::vector<ipc::VisionCameraDesc> table;
            publishedCameras.clear();
            std::unordered_map<std::uint32_t, unsigned> perVehicle;
            for (std::size_t i = 0; i < scene->cameraCount(); ++i) {
                if (!offscreen.active(static_cast<unsigned>(i))) continue;
                const std::uint32_t vehicleId = scene->cameraVehicle(i);
                ipc::VisionCameraDesc d;
                d.vehicleId = vehicleId;
                d.width = offscreen.width(static_cast<unsigned>(i));
                d.height = offscreen.height(static_cast<unsigned>(i));
                const Vehicle v = world->vehicle(vehicleId);
                d.label = v.valid() ? v.name() : "vehicle " + std::to_string(vehicleId);
                const unsigned n = perVehicle[vehicleId]++;
                if (n > 0) d.label += " #" + std::to_string(n + 1);
                table.push_back(std::move(d));
                publishedCameras.push_back(static_cast<unsigned>(i));
            }
            const unsigned kept = publisher.setCameras(table);
            publishedCameras.resize(kept);
            publishedTableVersion = cameraTableVersion;
        }
        for (unsigned slot = 0; slot < publishedCameras.size(); ++slot) {
            const auto& rgb = offscreen.rgb(publishedCameras[slot]);
            publisher.write(slot, rgb.data(), rgb.size());
        }
        publisher.endFrame();
    }

    /// The vehicles present now, as the scene draws them: each one's state, model and the stores on its stations.
    void syncVehicles() {
        std::vector<VehicleFrame> frames;
        for (const auto& v : world->vehicles()) {
            VehicleFrame f;
            f.id = v.id();
            f.model = v.model();
            f.type = v.type();
            f.state = v.state();
            f.cgFromModelOriginM = v.cgFromModelOriginM();
            for (const ms::StoreInfo& store : v.stores()) {
                f.doors.emplace_back(store.station, store.bayDoors);
                if (!*store.store || store.status == ms::StoreStatus::Releasing || store.status == ms::StoreStatus::Released ||
                    store.status == ms::StoreStatus::Jettisoned)
                    continue;
                f.carried.emplace_back(store.station, store.store);
            }
            frames.push_back(std::move(f));
        }
        scene->update(frames);
    }

    /// Each camera's vehicle where it is now (none when it is gone), and the world's clock for the sun.
    void aim() {
        std::vector<std::optional<sim::VehicleState>> poses(scene->cameraCount());
        for (std::size_t i = 0; i < poses.size(); ++i) {
            const Vehicle v = world->vehicle(scene->cameraVehicle(i));
            if (v.valid()) poses[i] = v.state();
        }
        scene->aim(poses, world->environment().utcSeconds());
    }
};

namespace {
[[noreturn]] void rethrow(const char* where, const vsg::Exception& e) {
    throw Error(std::string("vision: ") + where + ": " + e.message + " (" + std::to_string(e.result) + ")");
}
} // namespace

Sensors::Sensors(World& world, const Options& options) : impl_(std::make_unique<Impl>()) {
  try {
    impl_->world = &world;
    impl_->options = options;
    CameraSceneSettings s;
    s.debugLayer = options.debugLayer;
    s.sky = options.sky;
    s.earth = options.earth;
    s.imagery = options.imagery;
    s.imageryUrl = options.imageryUrl;
    s.maxLevel = options.maxLevel;
    s.elevation = options.elevation;
    s.elevationUrl = options.elevationUrl;
    s.assetDir = options.assetDir;
    s.segmentation = options.segmentation;
    s.maxVehicles = options.maxVehicles;
    try {
        impl_->scene = std::make_unique<CameraScene>(s, world.environment().utcSeconds());
    } catch (const CameraSceneError& e) {
        throw Error(e.what()); // (the offscreen device's failure: "vision: ...")
    }
  } catch (const vsg::Exception& e) {
    rethrow("scene", e);
  }
}

Sensors::~Sensors() = default;

unsigned Sensors::addCamera(const Vehicle& vehicle, const CameraSpec& spec) {
    CameraMount mount;
    mount.offsetBodyM = {spec.offsetBodyM[0], spec.offsetBodyM[1], spec.offsetBodyM[2]};
    mount.yawDeg = spec.yawDeg;
    mount.pitchDeg = spec.pitchDeg;
    mount.rollDeg = spec.rollDeg;
    mount.width = spec.width;
    mount.height = spec.height;
    mount.fovDeg = spec.fovDeg;
    mount.hideOwnVehicle = spec.hideOwnVehicle;
    mount.depth = spec.depth;
    mount.segmentation = spec.segmentation;
    ++impl_->cameraTableVersion;
    return impl_->scene->addCamera(vehicle.id(), mount);
}

std::size_t Sensors::cameraCount() const noexcept { return impl_->scene->cameraCount(); }

void Sensors::removeCamera(unsigned camera) {
    impl_->scene->offscreen().removeCamera(camera);
    ++impl_->cameraTableVersion;
}

void Sensors::render() {
    const auto t0 = std::chrono::steady_clock::now();
    try {
        if (impl_->scene->offscreen().activeCameraCount() == 0) throw Error("vision: no cameras");
        impl_->syncVehicles();
        impl_->aim();
        impl_->scene->offscreen().render(); // compiles first when cameras were added or removed
        impl_->compiled = true;
        impl_->publish();
    } catch (const vsg::Exception& e) {
        rethrow("render", e);
    }
    impl_->lastRenderMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

}

void Sensors::settle(unsigned frames) {
    if (!impl_->compiled) return;
    impl_->syncVehicles();
    impl_->aim();
    for (unsigned i = 0; i < frames; ++i) {
        impl_->scene->offscreen().advance();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    vkDeviceWaitIdle(*impl_->scene->offscreen().device());
}

Image Sensors::image(unsigned camera) const noexcept {
    Image img;
    if (!impl_->scene->offscreen().active(camera)) return img;
    img.rgb = impl_->scene->offscreen().rgb(camera).data();
    img.width = impl_->scene->offscreen().width(camera);
    img.height = impl_->scene->offscreen().height(camera);
    return img;
}

DepthImage Sensors::depth(unsigned camera) const noexcept {
    DepthImage img;
    if (!impl_->scene->offscreen().active(camera) || impl_->scene->offscreen().depth(camera).empty()) return img;
    img.metres = impl_->scene->offscreen().depth(camera).data();
    img.width = impl_->scene->offscreen().width(camera);
    img.height = impl_->scene->offscreen().height(camera);
    return img;
}

SegmentationImage Sensors::segmentation(unsigned camera) const noexcept {
    SegmentationImage img;
    if (!impl_->scene->offscreen().active(camera) || impl_->scene->offscreen().segmentation(camera).empty()) return img;
    img.ids = impl_->scene->offscreen().segmentation(camera).data();
    img.width = impl_->scene->offscreen().width(camera);
    img.height = impl_->scene->offscreen().height(camera);
    return img;
}

unsigned Sensors::segmentationId(const Vehicle& vehicle) const noexcept {
    return impl_->scene->segmentationId(vehicle.id());
}

bool Sensors::saveSegmentationPng(unsigned camera, const std::string& path) const {
    const SegmentationImage seg = segmentation(camera);
    if (!seg.ids) return false;
    // A readable palette rather than the raw id: golden-ratio hues, black for 0.
    std::vector<std::uint8_t> rgb(seg.size() * 3, 0);
    for (std::size_t i = 0; i < seg.size(); ++i) {
        const unsigned id = seg.ids[i];
        if (id == 0) continue;
        const double h = std::fmod(static_cast<double>(id) * 0.61803398875, 1.0) * 6.0;
        const int sector = static_cast<int>(h);
        const double f = h - sector;
        const double v = 1.0, p = 0.25, q = 1.0 - 0.75 * f, t = 0.25 + 0.75 * f;
        double r = v, g = v, b = v;
        switch (sector) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
        }
        rgb[i * 3 + 0] = static_cast<std::uint8_t>(r * 255.0 + 0.5);
        rgb[i * 3 + 1] = static_cast<std::uint8_t>(g * 255.0 + 0.5);
        rgb[i * 3 + 2] = static_cast<std::uint8_t>(b * 255.0 + 0.5);
    }
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const int ok = stbi_write_png_to_func([](void* ctx, void* data, int size) { std::fwrite(data, 1, static_cast<std::size_t>(size), static_cast<std::FILE*>(ctx)); }, f,
                                          static_cast<int>(seg.width), static_cast<int>(seg.height), 3, rgb.data(), static_cast<int>(seg.width) * 3);
    std::fclose(f);
    return ok != 0;
}

bool Sensors::saveDepthPng(unsigned camera, const std::string& path, double farM) const {
    const DepthImage d = depth(camera);
    if (!d.metres) return false;
    std::vector<std::uint8_t> grey(d.size());
    const double lf = std::log(std::max(2.0, farM));
    for (std::size_t i = 0; i < d.size(); ++i) {
        const double m = std::max(1.0, static_cast<double>(d.metres[i]));
        const double v = 1.0 - std::min(1.0, std::log(m) / lf);
        grey[i] = static_cast<std::uint8_t>(v * 255.0 + 0.5);
    }
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const int ok = stbi_write_png_to_func([](void* ctx, void* data, int size) { std::fwrite(data, 1, static_cast<std::size_t>(size), static_cast<std::FILE*>(ctx)); }, f,
                                          static_cast<int>(d.width), static_cast<int>(d.height), 1, grey.data(), static_cast<int>(d.width));
    std::fclose(f);
    return ok != 0;
}

bool Sensors::savePng(unsigned camera, const std::string& path) const {
    const Image img = image(camera);
    if (!img.rgb) return false;
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const int ok = stbi_write_png_to_func([](void* ctx, void* data, int size) { std::fwrite(data, 1, static_cast<std::size_t>(size), static_cast<std::FILE*>(ctx)); }, f,
                                          static_cast<int>(img.width), static_cast<int>(img.height), 3, img.rgb, static_cast<int>(img.width) * 3);
    std::fclose(f);
    return ok != 0;
}

double Sensors::lastRenderMs() const noexcept { return impl_->lastRenderMs; }
Sensors::Timing Sensors::timing() const noexcept {
    const auto& t = impl_->scene->offscreen().timing();
    return Timing{t.advance, t.update, t.record, t.wait, t.readback};
}
std::uint64_t Sensors::frames() const noexcept { return impl_->scene->offscreen().frames(); }

// --- BatchCameras -----------------------------------------------------------------------

struct BatchCameras::Impl {
    std::unique_ptr<Sensors> sensors;
    std::vector<unsigned> cameras; ///< per batch vehicle
    std::vector<std::uint8_t> rgb;
    std::vector<float> depth;
    std::vector<std::uint16_t> ids;
    unsigned width = 0, height = 0;
    bool wantDepth = false, wantSegmentation = false;
};

BatchCameras::BatchCameras(VecEnv& env, const CameraSpec& spec, const Options& options) : impl_(std::make_unique<Impl>()) {
    impl_->sensors = std::make_unique<Sensors>(env.world(), options);
    impl_->width = std::max(1u, spec.width);
    impl_->height = std::max(1u, spec.height);
    impl_->wantDepth = spec.depth;
    impl_->wantSegmentation = spec.segmentation && options.segmentation;
    // Vehicles are named "env<e>/<v>" in batch order; mount in that order.
    const unsigned K = env.vehiclesPerEnv();
    for (unsigned e = 0; e < env.numEnvs(); ++e)
        for (unsigned k = 0; k < K; ++k) {
            const Vehicle v = env.world().vehicle("env" + std::to_string(e) + "/" + std::to_string(k));
            if (!v.valid()) throw Error("vision: batch vehicle env" + std::to_string(e) + "/" + std::to_string(k) + " not found");
            impl_->cameras.push_back(impl_->sensors->addCamera(v, spec));
        }
    const std::size_t pixels = static_cast<std::size_t>(impl_->width) * impl_->height;
    impl_->rgb.assign(impl_->cameras.size() * pixels * 3, 0);
    if (impl_->wantDepth) impl_->depth.assign(impl_->cameras.size() * pixels, 0.0f);
    if (impl_->wantSegmentation) impl_->ids.assign(impl_->cameras.size() * pixels, 0);
}

BatchCameras::~BatchCameras() = default;

void BatchCameras::render() {
    impl_->sensors->render();
    const std::size_t pixels = static_cast<std::size_t>(impl_->width) * impl_->height;
    for (std::size_t i = 0; i < impl_->cameras.size(); ++i) {
        const Image img = impl_->sensors->image(impl_->cameras[i]);
        if (img.rgb) std::memcpy(impl_->rgb.data() + i * pixels * 3, img.rgb, pixels * 3);
        if (impl_->wantDepth) {
            const DepthImage d = impl_->sensors->depth(impl_->cameras[i]);
            if (d.metres) std::memcpy(impl_->depth.data() + i * pixels, d.metres, pixels * sizeof(float));
        }
        if (impl_->wantSegmentation) {
            const SegmentationImage seg = impl_->sensors->segmentation(impl_->cameras[i]);
            if (seg.ids) std::memcpy(impl_->ids.data() + i * pixels, seg.ids, pixels * sizeof(std::uint16_t));
        }
    }
}

ConstSpan<std::uint8_t> BatchCameras::rgb() const noexcept { return ConstSpan<std::uint8_t>{impl_->rgb.data(), impl_->rgb.size()}; }
ConstSpan<float> BatchCameras::depth() const noexcept { return ConstSpan<float>{impl_->depth.data(), impl_->depth.size()}; }
ConstSpan<std::uint16_t> BatchCameras::segmentation() const noexcept { return ConstSpan<std::uint16_t>{impl_->ids.data(), impl_->ids.size()}; }
std::size_t BatchCameras::count() const noexcept { return impl_->cameras.size(); }
unsigned BatchCameras::width() const noexcept { return impl_->width; }
unsigned BatchCameras::height() const noexcept { return impl_->height; }
Sensors& BatchCameras::sensors() noexcept { return *impl_->sensors; }
unsigned BatchCameras::camera(std::size_t vehicle) const noexcept { return vehicle < impl_->cameras.size() ? impl_->cameras[vehicle] : 0u; }

} // namespace fsim::vision
