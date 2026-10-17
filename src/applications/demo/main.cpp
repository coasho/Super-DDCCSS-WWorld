// flightsim-demo: the demo publisher (ARCH-08, HC-16). The viewer's built-in demo - a few native aircraft flown by a
// small autopilot (DemoSimulation, unchanged) - in a process of its own, published on the read-only world stream like any
// world, so the live/replay viewer composes no simulation. `flightsim-viewer --demo` launches it with its own demo
// options and drives its simulation thread through the demo control block (contracts/world_stream/DemoControl.h):
// pause, single step, time factor and quit. Run on its own it publishes until stopped.
//
// Usage: flightsim-demo --world NAME [--control] [--parent-pid PID] [demo options] [terrain options]

#include "applications/demo/DemoSimulation.h"
#include "contracts/world_stream/DemoControl.h"
#include "contracts/world_stream/WorldLayout.h"
#include "foundation/core/Log.h"
#include "foundation/io/AssetResolver.h"
#include "foundation/io/TerrainTiles.h"
#include "foundation/platform/CrashHandler.h"
#include "foundation/platform/SharedMemory.h"
#include "foundation/transport/WorldPublisher.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace fsim;

namespace {

struct PublisherOptions {
    app::DemoOptions demo;
    std::string world = "demo";
    bool control = false;              ///< open the viewer's control block (demoControlName(world))
    std::uint64_t parentPid = 0;       ///< exit when this process (the viewer) has gone
    std::vector<std::string> assetDirs; ///< --assets, highest priority first
    std::string elevationUrl;          ///< the physics' ground: empty for flat ground (--elevation none)
    unsigned terrainZoom = 15;
    std::filesystem::path tileCache;
    bool offline = true;
};

void usage() {
    std::fprintf(stderr,
                 "usage: flightsim-demo --world NAME [--control] [--parent-pid PID]\n"
                 "         [--aircraft NAME] [--jsbsim-root DIR] [--assets DIR]... [--vehicles N] [--workers N]\n"
                 "         [--dt S] [--frame-skip N] [--seed N] [--throttle X] [--speed M/S] [--time-factor X]\n"
                 "         [--lat DEG] [--lon DEG] [--alt M] [--spread DEG] [--on-ground]\n"
                 "         [--elevation URL|none] [--terrain-zoom N] [--tile-cache DIR] [--offline|--online]\n"
                 "         [--log-level trace|debug|info|warn|error]\n");
}

bool parse(int argc, char** argv, PublisherOptions& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + a);
            return argv[++i];
        };
        try {
            if (a == "--world") o.world = next();
            else if (a == "--control") o.control = true;
            else if (a == "--parent-pid") o.parentPid = std::stoull(next());
            else if (a == "--aircraft") o.demo.aircraft = next();
            else if (a == "--jsbsim-root") o.demo.jsbsimRoot = next();
            else if (a == "--assets") o.assetDirs.push_back(next());
            else if (a == "--vehicles") o.demo.vehicles = static_cast<unsigned>(std::stoul(next()));
            else if (a == "--workers") o.demo.workers = static_cast<unsigned>(std::stoul(next()));
            else if (a == "--dt") o.demo.dt = std::stod(next());
            else if (a == "--frame-skip") o.demo.frameSkip = std::stoi(next());
            else if (a == "--seed") o.demo.seed = std::stoull(next());
            else if (a == "--throttle") o.demo.throttle = std::stod(next());
            else if (a == "--speed") o.demo.speedMs = std::stod(next());
            else if (a == "--time-factor") o.demo.timeFactor = std::stod(next());
            else if (a == "--lat") o.demo.latitudeDeg = std::stod(next());
            else if (a == "--lon") o.demo.longitudeDeg = std::stod(next());
            else if (a == "--alt") o.demo.altitudeM = std::stod(next());
            else if (a == "--spread") o.demo.spreadDeg = std::stod(next());
            else if (a == "--on-ground") o.demo.onGround = true;
            else if (a == "--elevation") {
                const std::string v = next();
                o.elevationUrl = v == "none" ? std::string() : v;
            } else if (a == "--terrain-zoom") o.terrainZoom = static_cast<unsigned>(std::stoul(next()));
            else if (a == "--tile-cache") o.tileCache = next();
            else if (a == "--offline") o.offline = true;
            else if (a == "--online") o.offline = false;
            else if (a == "--log-level") {
                const std::string v = next();
                log::setLevel(v == "trace" ? log::Level::Trace : v == "debug" ? log::Level::Debug : v == "warn" ? log::Level::Warn
                              : v == "error" ? log::Level::Error : log::Level::Info);
            } else {
                std::fprintf(stderr, "unknown option %s\n", a.c_str());
                return false;
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "bad argument: %s\n", e.what());
            return false;
        }
    }
    return o.demo.vehicles >= 1 && !o.world.empty();
}

} // namespace

int main(int argc, char** argv) {
    platform::installCrashHandler();
    PublisherOptions opt;
    if (!parse(argc, argv, opt)) {
        usage();
        return 2;
    }

    io::AssetResolver assets;
    for (auto it = opt.assetDirs.rbegin(); it != opt.assetDirs.rend(); ++it) assets.addSearchPath(*it);
    // the physics' ground: the viewer's own terrain rule (Terrarium elevation tiles at its terrain zoom, its map set)
    std::shared_ptr<io::TerrainTiles> terrain;
    if (!opt.elevationUrl.empty()) {
        io::TerrainTiles::Options to;
        to.urlTemplate = opt.elevationUrl;
        to.zoom = opt.terrainZoom;
        to.cacheDir = opt.tileCache;
        to.offline = opt.offline;
        terrain = std::make_shared<io::TerrainTiles>(to);
    }
    auto runner = app::DemoSimulation::create(opt.demo, assets, terrain);
    if (!runner) return 1;
    const auto vehicles = static_cast<std::uint32_t>(runner->vehicles());

    // the viewer's control block, made and initialised by the viewer before it launched this process
    platform::SharedMemory controlMemory;
    ipc::DemoControl* control = nullptr;
    if (opt.control) {
        if (!controlMemory.open(ipc::demoControlName(opt.world), sizeof(ipc::DemoControl))) {
            LOG_ERROR("demo") << "no control block '" << ipc::demoControlName(opt.world) << "'";
            return 1;
        }
        control = static_cast<ipc::DemoControl*>(controlMemory.data());
        if (control->magic != ipc::kDemoControlMagic || control->version != ipc::kDemoControlVersion) {
            LOG_ERROR("demo") << "control block '" << ipc::demoControlName(opt.world) << "' has another layout";
            return 1;
        }
    }

    ipc::WorldPublisher::Options po;
    po.name = opt.world;
    po.capacity = vehicles;
    po.dt = opt.demo.dt;
    po.frameSkip = opt.demo.frameSkip;
    ipc::WorldPublisher publisher(po);
    if (!publisher.active()) {
        LOG_ERROR("demo") << "cannot publish world '" << opt.world << "'";
        return 1;
    }
    // Its vehicles, all of one type, and each one's CG's place from its model's origin: read once, before the
    // simulation's thread runs, as the in-process demo read it (the viewer hangs each picture from its origin).
    std::vector<ipc::FlightStatus> status(vehicles);
    for (std::uint32_t i = 0; i < vehicles; ++i) {
        ipc::VehicleRecord row;
        row.id = i + 1;
        row.alive = 1;
        const std::string name = "demo " + std::to_string(i + 1), type = "jsbsim:" + opt.demo.aircraft;
        std::strncpy(row.name, name.c_str(), sizeof row.name - 1);
        std::strncpy(row.type, type.c_str(), sizeof row.type - 1);
        row.initialLatitudeDeg = opt.demo.latitudeDeg;
        row.initialLongitudeDeg = opt.demo.longitudeDeg;
        row.initialAltitudeMslM = opt.demo.altitudeM;
        publisher.setVehicle(i, row);
        runner->cgFromOriginBodyM(i, status[i].derived + ipc::kDerivedCgFromOriginM);
    }
    LOG_INFO("demo") << "publishing world '" << opt.world << "': " << vehicles << " x " << opt.demo.aircraft;

    // the viewer's commands as they stand before the simulation's thread runs (a paused start steps nothing)
    if (control) {
        runner->setPaused(control->paused.load() != 0);
        runner->setTimeFactor(control->timeFactor.load());
    }
    runner->start();
    std::uint64_t stepsSeen = control ? control->stepRequests.load() : 0;
    while (!(control && control->quit.load(std::memory_order_relaxed))) {
        if (control) {
            runner->setPaused(control->paused.load(std::memory_order_relaxed) != 0);
            runner->setTimeFactor(control->timeFactor.load(std::memory_order_relaxed));
            for (const std::uint64_t steps = control->stepRequests.load(std::memory_order_relaxed); stepsSeen < steps; ++stepsSeen)
                runner->singleStep();
            control->throughput.store(runner->throughput(), std::memory_order_relaxed);
        }
        if (const auto* fresh = runner->acquire())
            publisher.publish(fresh->simTime, Span<const sim::VehicleState>(fresh->states), {}, true, Span<const ipc::FlightStatus>(status));
        if (opt.parentPid && !platform::processAlive(opt.parentPid)) {
            LOG_INFO("demo") << "the viewer has gone; stopping";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    runner->stop();
    return 0;
}
