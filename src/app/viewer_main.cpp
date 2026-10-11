// flightsim-viewer.exe - the prebuilt visualisation application (design 9.1,
// 9.7, 9.10).
//
// Default ("mirror") mode: discover a training application's world through
// shared memory, attach, and mirror its vehicles: creations, resets, state,
// control level and environment. The training process never knows the
// viewer exists. `--demo` runs the built-in scenario on an internal simulation
// thread instead (development, screenshots).

#include "app/DemoAutopilot.h"

#include <fstream>
#include <map>
#include <optional>
#include <cstdlib>
#include "core/Log.h"
#include "core/Rng.h"
#include "core/Units.h"
#include "fsim/Control.h"
#include "io/AssetResolver.h"
#include "io/AircraftFile.h"
#include "io/TerrainTiles.h"
#include "fsim/MissionSystems.h"
#include "ipc/Recording.h"
#include "ipc/WorldMirror.h"
#include "ipc/WorldRegistry.h"
#include "core/Json.h"
#include "platform/Clock.h"
#include "platform/Paths.h"
#include "platform/CrashHandler.h"
#include "platform/Threads.h"
#include "render/GpuTimer.h"
#include "render/Viewer.h"
#include "world/Atmosphere.h"
#include "sim/GroundProvider.h"
#include "sim/JsbsimModel.h"
#include "sim/SimRunner.h"
#include "sim/VehiclePool.h"
#include "ui/KeyHandler.h"
#include "ui/CameraPreview.h"
#include "ui/MissionGui.h"
#include "ui/MonitorGui.h"
#include "ui/ViewerControls.h"
#include "world/Airflow.h"
#include "world/WingVapour.h"
#include "world/Detonations.h"
#include "world/FlowVolumes.h"
#include "world/CameraController.h"
#include "world/Clouds.h"
#include "world/Earth.h"
#include "world/Interpolator.h"
#include "world/Sky.h"
#include "world/SkyDome.h"
#include "world/Terrain.h"
#include "world/Trails.h"
#include "world/VehicleVisuals.h"

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/imgui.h>
#include <vsgImGui/SendEventsToImGui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

using namespace fsim;

namespace {
/// A vehicle's state as its picture is hung: its position its model's origin (ipc::originEcef), not its CG's - the
/// point its model, its stations and its parts are measured from, which the simulation keeps still in its airframe
/// while its CG moves with its fuel and its stores. Its other values (its latitude, its height: what the panels show)
/// are its state's own.
sim::VehicleState drawnState(const sim::VehicleState& state, const double* derived) {
    sim::VehicleState drawn = state;
    ipc::originEcef(state, derived, drawn.positionEcef);
    return drawn;
}


struct ViewerOptions {
    // Source
    bool demo = false;
    bool list = false;
    std::string worldName;       // --world: attach to this world (default: the newest)
    std::string replayPath;      // --replay: play a recording instead of a live world
    unsigned capacity = 256;     // slots to prepare before a world is attached

    // Demo scenario
    std::string aircraft = "c172x";
    std::filesystem::path jsbsimRoot;
    std::vector<std::string> assetDirs; // --assets, highest priority first
    unsigned vehicles = 8;
    unsigned workers = 0;
    double dt = 1.0 / 120.0;
    int frameSkip = 2; // 60 Hz agent rate for smooth motion
    std::uint64_t seed = 1;
    double throttle = 0.65;
    double speedMs = 0.0; // 0: from the aircraft's wing loading
    double timeFactor = 1.0;
    double latitudeDeg = 37.6188, longitudeDeg = -122.375, altitudeM = 1500.0, spreadDeg = 0.03;
    bool onGround = false;

    // Scene
    world::EarthSettings earth;
    unsigned terrainZoom = 15; ///< the raster drawn close up (earth.elevationMaxLevel)
    /// Where map assets live. Empty = the local map set beside the program (platform::mapsDir()).
    std::filesystem::path tileCache;
    /// Draw only what is already in tileCache and never fetch a tile (the
    /// default; --online, or map.offline false, lets it download).
    bool offline = true;
    bool gui = true;   ///< panels, labels and trails; --no-gui leaves only the rendered scene
    bool effects = true; ///< exhaust, condensation, vapour and contrails (--no-effects, key e)
    std::string fire = "flow"; ///< --fire flow|spheres: an engagement's fire and smoke solved by NVIDIA Flow, or the fireballs
    std::string clouds = "world"; ///< --clouds: the world's weather, or a sky of our own (key k)
    double seaWindMs = -1.0;      ///< --sea-wind: the wind the sea is made for, whatever the weather says (< 0: the weather's)
    bool zoomToCursor = true; ///< wheel zooms towards the pointer rather than straight in
    bool sun = true;
    double sunUtcHours = -1.0;
    std::string modelPath;
    std::string modelForward, modelUp;
    double modelScaleOverride = 0.0;

    render::ViewerSettings window;
    int cameraMode = 1; // orbit
    std::string view;   // --view lat,lon,alt_m,distance_m[,azimuth_deg,elevation_deg]: start detached, looking at that point
    double chaseDistance = 40.0;
    bool chaseDistanceSet = false; // --chase-distance given: start there, even attached to a world
    double chaseAzimuth = 180.0, chaseElevation = 14.0;
    int follow = 0; // --follow <slot>: the vehicle the camera starts on (tab moves on from it)
    bool chaseElevationSet = false; // --chase-elevation given: keep it, even attached to a world
    bool probe = false;
    double stats = 0.0;   // --stats <seconds>: print per-second frame statistics, exit after this long
    std::string screenshot;      // --screenshot <file.png>: save the frame after --screenshot-after seconds, then exit
    double screenshotAfter = 3.0;
    int screenshotFrames = 1;    // --screenshot-frames <n>: n frames, <file>_000.png on, --screenshot-every <s> apart
    double screenshotEvery = 0.1;
    bool screenshotReplayClock = false; // --screenshot-clock replay: --screenshot-after/-every in the replay's time, each frame drawn at its time
    double screenshotZoom = 1.0; // --screenshot-zoom <x>: the camera's distance times x a second from a second before the first frame
    double screenshotOrbit = 0.0; // --screenshot-orbit <deg/s>: the camera turned round what it looks at, as a drag turns it
    int trace = -1;
    bool interpolate = true;
    bool help = false;
    bool helpAll = false; ///< --help-all: the diagnostic and scripting options too
};

/// What someone running the viewer needs, which is not the same as what it
/// can do. Forty options in one list is not a interface, it is an inventory;
/// the rest are for scripting and for working on the viewer, and --help-all
/// still prints them.
void usage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "\n"
        "  --demo                   fly the built-in scenario instead of attaching to a trainer\n"
        "  --replay <file.fsrec>    play back a recording\n"
        "  --list                   list live worlds and exit\n"
        "\n"
        "  --view lat,lon,alt,dist[,az,el]   start looking at that place\n"
        "  --camera chase|orbit|overview|free\n"
        "  --aircraft <name>        JSBSim aircraft for --demo (c172x)\n"
        "  --vehicles <n>           how many (8)\n"
        "\n"
        "  --fullscreen  --width <px>  --height <px>\n"
        "  --no-gui                 just the rendered scene, no panels or labels\n"
        "  --no-effects             no exhaust, condensation, vapour or contrails (key e)\n"
        "  --clouds <sky>           world (its weather; default), clear, few, scattered, broken, overcast, rain, off (key k)\n"
        "  --screenshot <file.png>  save a frame and exit\n"
        "\n"
        "  --config <file>          settings file (default: <exe>/../config/viewer.json)\n"
        "  --online                 also download tiles the local map set lacks (default: never)\n"
        "\n"
        "  --help-all               every option, including the diagnostic ones\n"
        "\n"
        "Most of what the flags reach is in config/viewer.json - window size, map\n"
        "sources, where the tiles live - and a flag overrides the file for one run.\n"
        "\n"
        "Keys: space pause, . step, tab next vehicle, c camera, -/= zoom, r reset view,\n"
        "      [ ] time factor, l list, m monitor, n labels, t trails, e effects, k clouds, esc quit\n"
        "Mouse: left drag rotates, space + left drag pans (not while locked to a vehicle), wheel zooms towards the pointer\n",
        prog);
}

/// Everything, for scripting and for working on the viewer itself.
void usageAll(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "Mirror mode (default): attach to a training application's world through shared memory.\n"
        "  --world <name>           attach to this world (default: the newest published one)\n"
        "  --list                   list live worlds and exit\n"
        "  --capacity <n>           vehicle slots to prepare (256)\n"
        "  --replay <file.fsrec>    play back a recording made with WorldOptions::recordPath (space pause, . step, [ ] speed, timeline seek, home)\n"
        "Demo mode: run the built-in scenario on an internal simulation thread.\n"
        "  --demo                   enable demo mode\n"
        "  --aircraft <name>        JSBSim aircraft (c172x)\n"
        "  --jsbsim-root <dir>      JSBSim data tree (auto)\n"
        "  --assets <dir>           extra asset directory (models/<type>.glb per vehicle type), searched first\n"
        "  --vehicles <n>           number of vehicles (8)\n"
        "  --workers <n>            sim workers (auto)\n"
        "  --time-factor <x>        simulation speed (1.0)\n"
        "  --lat <deg> --lon <deg>  spawn centre (KSFO)\n"
        "  --alt <m>                spawn altitude MSL (1500)\n"
        "  --speed <m/s>            spawn airspeed (auto: its weight carried at CL 0.5, at least 58)\n"
        "  --spread <deg>           spawn scatter (0.03)\n"
        "  --on-ground              spawn parked on the terrain (brakes on)\n"
        "  --terrain-zoom <z>       tile level used for physics ground height (15, the level drawn close up)\n"
        "  --probe                  print motion smoothness statistics after ~5 s and exit\n"
        "  --stats <seconds>        print per-second frame statistics (fps, frame-time breakdown, CPU; the GPU's time per pass) and exit\n"
        "  --fire flow|spheres      an engagement's fire and smoke: solved by NVIDIA Flow (the default, when built with it) or\n"
        "                           drawn as fireballs and puffs\n"
        "  --screenshot <file.png>  save the window after --screenshot-after seconds (3) and exit\n"
        "  --screenshot-frames <n>  save n frames instead, <file>_000.png on, --screenshot-every <s> (0.1) apart\n"
        "  --screenshot-clock wall|replay  what --screenshot-after and -every count: the window's seconds (wall, the\n"
        "                           default) or a replay's own time, each frame drawn at its time however long saving takes\n"
        "  --screenshot-zoom <x>    zoom while they are saved: the camera's distance times x a second (0.5 in, 2 out),\n"
        "                           from a second before the first\n"
        "  --screenshot-orbit <deg/s>  turn the camera round what it looks at while they are saved, as dragging it does\n"
        "  --view lat,lon,alt,dist[,az,el]  start with the free camera looking at that point from dist metres (az deg from north, el deg up; default 180, 45)\n"
        "  --trace <i>              print vehicle i's state once per second\n"
        "  --no-interpolate         draw raw snapshots (sample-and-hold) instead of interpolating\n"
        "Scene:\n"
        "  --imagery satellite|osm|bing|none|<url template with {z}/{x}/{y}>   (satellite = Esri World Imagery)\n"
        "  --elevation terrarium|none|<url template>   relief from Terrarium-encoded tiles (default terrarium)\n"
        "  --config <file>          settings file (default: <exe>/../config/viewer.json; flags win)\n"
        "  --tile-cache <dir>       where map tiles are read (overrides map.tileCache; default: maps/ beside the program)\n"
        "  --online                 also download tiles the map set lacks, into it (default: offline, never fetch)\n"
        "  --no-zoom-to-cursor      the wheel zooms straight in rather than towards the pointer "
        "(osgEarth zoomToMouse is on by default)\n"
        "  --no-gui                 no panels, labels or trails: just the rendered scene "
        "(for looking at the graphics)\n"
        "  --no-effects             no exhaust flames, condensation, vapour or contrails (config \"effects\"; key e)\n"
        "  --clouds <sky>           world: the weather the world publishes (default); or clear, few, scattered,\n"
        "                           broken, overcast, rain - a sky of its own, whatever the world says; off: none (config \"clouds\"; key k)\n"
        "  --sea-wind <m/s>         the wind the sea's waves are made for, whatever the weather says (looking at the sea)\n"
        "  --no-sun                 headlight instead of sun + ambient lighting\n"
        "  --sun-utc <hours>        sun position for this UTC hour (default: the world's time)\n"
        "  --bing-key <key>         Bing Maps key for --imagery bing\n"
        "  --max-level <n>          custom pyramid max level (17)\n"
        "  --elevation-max-level <n>  deepest elevation level to ask for (15); match an\n"
        "offline package to what tile_prefetch put in it\n"
        "  --model <file>           glTF/OBJ vehicle model (placeholder if omitted)\n"
        "  --model-scale <x>        model scale (1.0)\n"
        "  --width <px> --height <px> --fullscreen --msaa <1|2|4|8> --fov <deg> --max-fps <n> (0 = vsync only)\n"
        "  --debug-layer            Vulkan validation layer\n"
        "  --camera chase|orbit|overview|free   initial camera (orbit)\n"
        "  --chase-distance <m>     start this far from the vehicle (40; attached to a world the view\n"
        "                           starts regional unless this is given)\n"
        "  --chase-azimuth <deg>  --chase-elevation <deg>   where the camera sits (180 = behind, 14 up)\n"
        "  --follow <slot>          start on that vehicle, or go to it when it appears (its slot: 0 the first; a showcase's missile, 1, from its launch)\n"
        "  --log-level <lvl>\n"
        "Keys: space pause (demo), . step (demo), tab next vehicle, c camera, -/= zoom, r reset view, [ ] time factor (demo),\n"
        "      l list, m monitor, n labels, t trails, e effects, k clouds, esc quit\n"
        "Mouse: left drag rotates, wheel zooms (6 m .. whole Earth), space + left drag pans - drags the globe in free mode or\n"
        "      while no vehicle exists; locked to a vehicle the camera only rotates; the eye never goes below the terrain\n",
        prog);
}

/// The configuration file: --config <path>, else <exe>/../config/viewer.json.
/// That one relative path is the package layout and the build tree alike -
/// bin/ next to config/ - so there is nothing to keep in step between them.
std::filesystem::path configPath(int argc, char** argv) {
    for (int i = 1; i + 1 < argc; ++i)
        if (std::string(argv[i]) == "--config") return argv[i + 1];
    std::error_code ec;
    const auto beside = platform::executableDir() / ".." / "config" / "viewer.json";
    if (std::filesystem::exists(beside, ec)) return std::filesystem::weakly_canonical(beside, ec);
    return {};
}

/// Defaults from the configuration file. Applied before the command line is
/// read, so a flag always wins over the file and the file always wins over the
/// built-in default. Anything absent, misspelt or of the wrong shape leaves
/// the default alone: a viewer that will not start because one line of JSON is
/// wrong is worse than one that starts with a 1600x900 window.
bool applyConfig(ViewerOptions& o, const std::filesystem::path& file) {
    std::ifstream in(file, std::ios::binary);
    if (!in) return false;
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    core::Json doc;
    try {
        doc = core::Json::parse(text, file.string());
    } catch (const std::exception& e) {
        LOG_WARN("app") << "ignoring " << file.string() << ": " << e.what();
        return false;
    }

    const auto& window = doc.child("window");
    o.window.width = static_cast<std::uint32_t>(window.number("width", o.window.width));
    o.window.height = static_cast<std::uint32_t>(window.number("height", o.window.height));
    o.window.fullscreen = window.boolean("fullscreen", o.window.fullscreen);
    o.window.fieldOfViewDeg = window.number("fovDeg", o.window.fieldOfViewDeg);
    o.window.maxFps = window.number("maxFps", o.window.maxFps);
    if (window.has("msaa")) {
        switch (static_cast<int>(window.number("msaa", 4))) {
        case 1: o.window.samples = VK_SAMPLE_COUNT_1_BIT; break;
        case 2: o.window.samples = VK_SAMPLE_COUNT_2_BIT; break;
        case 4: o.window.samples = VK_SAMPLE_COUNT_4_BIT; break;
        case 8: o.window.samples = VK_SAMPLE_COUNT_8_BIT; break;
        default: LOG_WARN("app") << "window.msaa must be 1, 2, 4 or 8; keeping the default"; break;
        }
    }

    const auto& map = doc.child("map");
    if (map.has("imagery")) {
        const std::string v = map.string("imagery", "satellite");
        using S = world::EarthSettings::Source;
        if (v == "osm") o.earth.source = S::OpenStreetMap;
        else if (v == "esri" || v == "satellite") o.earth.source = S::EsriWorldImagery;
        else if (v == "bing") o.earth.source = S::Bing;
        else if (v == "none") o.earth.source = S::None;
        else { o.earth.source = S::Custom; o.earth.imageryUrl = v; }
    }
    if (map.has("elevation")) {
        const std::string v = map.string("elevation", "terrarium");
        if (v == "terrarium") o.earth.elevationUrl = world::kAwsTerrariumUrl;
        else if (v == "none") o.earth.elevationUrl.clear();
        else o.earth.elevationUrl = v;
    }
    o.earth.maxLevel = static_cast<unsigned>(map.number("maxLevel", o.earth.maxLevel));
    o.earth.elevationMaxLevel = static_cast<unsigned>(map.number("elevationMaxLevel", o.earth.elevationMaxLevel));
    if (map.has("tileCache")) {
        const std::string v = map.string("tileCache", "");
        if (!v.empty()) {
            std::error_code ec;
            const std::filesystem::path p(v);
            // Relative to the configuration file, so a package that is moved
            // or copied still finds the tiles it brought with it.
            o.tileCache = p.is_absolute() ? p : std::filesystem::weakly_canonical(file.parent_path() / p, ec);
        }
    }

    o.offline = map.boolean("offline", o.offline);
    o.zoomToCursor = doc.child("camera").boolean("zoomToCursor", o.zoomToCursor);
    o.gui = doc.boolean("gui", o.gui);
    o.effects = doc.boolean("effects", o.effects);
    o.clouds = doc.string("clouds", o.clouds);
    return true;
}

bool parse(int argc, char** argv, ViewerOptions& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + a);
            return argv[++i];
        };
        try {
            if (a == "-h" || a == "--help") o.help = true;
            else if (a == "--help-all") o.helpAll = o.help = true;
            else if (a == "--demo") o.demo = true;
            else if (a == "--list") o.list = true;
            else if (a == "--world") o.worldName = next();
            else if (a == "--replay") o.replayPath = next();
            else if (a == "--capacity") o.capacity = static_cast<unsigned>(std::stoul(next()));
            else if (a == "--aircraft") o.aircraft = next();
            else if (a == "--jsbsim-root") o.jsbsimRoot = next();
            else if (a == "--assets") o.assetDirs.push_back(next());
            else if (a == "--vehicles") o.vehicles = static_cast<unsigned>(std::stoul(next()));
            else if (a == "--workers") o.workers = static_cast<unsigned>(std::stoul(next()));
            else if (a == "--time-factor") o.timeFactor = std::stod(next());
            else if (a == "--lat") o.latitudeDeg = std::stod(next());
            else if (a == "--lon") o.longitudeDeg = std::stod(next());
            else if (a == "--alt") o.altitudeM = std::stod(next());
            else if (a == "--spread") o.spreadDeg = std::stod(next());
            else if (a == "--seed") o.seed = std::stoull(next());
            else if (a == "--throttle") o.throttle = std::stod(next());
            else if (a == "--speed") o.speedMs = std::stod(next());
            else if (a == "--imagery") {
                const std::string v = next();
                using S = world::EarthSettings::Source;
                if (v == "osm") o.earth.source = S::OpenStreetMap;
                else if (v == "esri" || v == "satellite") o.earth.source = S::EsriWorldImagery;
                else if (v == "bing") o.earth.source = S::Bing;
                else if (v == "none") o.earth.source = S::None;
                else { o.earth.source = S::Custom; o.earth.imageryUrl = v; }
            } else if (a == "--elevation") {
                const std::string v = next();
                if (v == "terrarium") o.earth.elevationUrl = world::kAwsTerrariumUrl;
                else if (v == "none") o.earth.elevationUrl.clear();
                else o.earth.elevationUrl = v;
            } else if (a == "--terrain-zoom") o.terrainZoom = static_cast<unsigned>(std::stoul(next()));
            else if (a == "--config") next(); // already read, before the command line
            else if (a == "--tile-cache") o.tileCache = next();
            else if (a == "--offline") o.offline = true;
            else if (a == "--online") o.offline = false;
            else if (a == "--no-zoom-to-cursor") o.zoomToCursor = false;
            else if (a == "--no-gui") o.gui = false;
            else if (a == "--no-effects") o.effects = false;
            else if (a == "--fire") o.fire = next();
            else if (a == "--clouds") o.clouds = next();
            else if (a == "--sea-wind") o.seaWindMs = std::atof(next());
            else if (a == "--no-sun") o.sun = false;
            else if (a == "--sun-utc") o.sunUtcHours = std::stod(next());
            else if (a == "--on-ground") o.onGround = true;
            else if (a == "--bing-key") o.earth.bingKey = next();
            else if (a == "--max-level") o.earth.maxLevel = static_cast<unsigned>(std::stoul(next()));
            else if (a == "--elevation-max-level") o.earth.elevationMaxLevel = static_cast<unsigned>(std::stoul(next()));
            else if (a == "--no-elevation-upsample") o.earth.upsampleElevation = false;
            else if (a == "--skirt") o.earth.skirtRatio = std::stod(next());
            else if (a == "--model") o.modelPath = next();
            else if (a == "--model-scale") o.modelScaleOverride = std::stod(next());
            else if (a == "--model-forward") o.modelForward = next();
            else if (a == "--model-up") o.modelUp = next();
            else if (a == "--width") o.window.width = static_cast<std::uint32_t>(std::stoul(next()));
            else if (a == "--height") o.window.height = static_cast<std::uint32_t>(std::stoul(next()));
            else if (a == "--fullscreen") o.window.fullscreen = true;
            else if (a == "--fov") o.window.fieldOfViewDeg = std::stod(next());
            else if (a == "--max-fps") o.window.maxFps = std::stod(next());
            else if (a == "--msaa") {
                const int s = std::stoi(next());
                o.window.samples = s >= 8 ? VK_SAMPLE_COUNT_8_BIT : s >= 4 ? VK_SAMPLE_COUNT_4_BIT : s >= 2 ? VK_SAMPLE_COUNT_2_BIT : VK_SAMPLE_COUNT_1_BIT;
            } else if (a == "--debug-layer") o.window.debugLayer = true;
            else if (a == "--chase-distance") { o.chaseDistance = std::stod(next()); o.chaseDistanceSet = true; }
            else if (a == "--chase-azimuth") o.chaseAzimuth = std::stod(next());
            else if (a == "--follow") o.follow = std::max(0, std::stoi(next()));
            else if (a == "--chase-elevation") { o.chaseElevation = std::stod(next()); o.chaseElevationSet = true; }
            else if (a == "--view") { o.view = next(); o.cameraMode = 3; }
            else if (a == "--camera") {
                const std::string v = next();
                o.cameraMode = v == "orbit" ? 1 : v == "overview" ? 2 : v == "free" ? 3 : 0;
            } else if (a == "--probe") o.probe = true;
            else if (a == "--stats") o.stats = std::stod(next());
            else if (a == "--screenshot") o.screenshot = next();
            else if (a == "--screenshot-after") o.screenshotAfter = std::stod(next());
            else if (a == "--screenshot-frames") o.screenshotFrames = std::max(1, std::stoi(next()));
            else if (a == "--screenshot-every") o.screenshotEvery = std::max(0.0, std::stod(next()));
            else if (a == "--screenshot-clock") o.screenshotReplayClock = std::string(next()) == "replay";
            else if (a == "--screenshot-zoom") o.screenshotZoom = std::clamp(std::stod(next()), 0.01, 100.0);
            else if (a == "--screenshot-orbit") o.screenshotOrbit = std::stod(next());
            else if (a == "--trace") o.trace = std::stoi(next());
            else if (a == "--no-interpolate") o.interpolate = false;
            else if (a == "--log-level") {
                const std::string v = next();
                log::setLevel(v == "trace" ? log::Level::Trace : v == "debug" ? log::Level::Debug : v == "warn" ? log::Level::Warn : v == "error" ? log::Level::Error : log::Level::Info);
            } else {
                std::fprintf(stderr, "unknown option %s\n", a.c_str());
                return false;
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "bad argument: %s\n", e.what());
            return false;
        }
    }
    return o.vehicles >= 1;
}

/// Environment one-liner for the monitor.
std::string describe(const sim::EnvironmentState& e, double simTime) {
    int day = 0; double hours = 0.0;
    world::utcOf(e.epochUtcSeconds + simTime, day, hours);
    char buf[256];
    std::snprintf(buf, sizeof buf, "UTC day %d %02d:%02d  wind %.0f deg / %.1f m/s  turb %.2f  T0 %.1f K  p0 %.0f hPa  vis %.0f km",
                  day, static_cast<int>(hours), static_cast<int>((hours - std::floor(hours)) * 60.0), e.windDirectionDeg, e.windSpeedMs,
                  e.turbulence, e.temperatureSeaLevelK, e.pressureSeaLevelPa / 100.0, e.visibilityM / 1000.0);
    return buf;
}

} // namespace

int main(int argc, char** argv) {
    platform::installCrashHandler(); // minidump under logs/crash beside the program on a crash
    // GUI subsystem: print to the terminal we were started from, else keep a log file.
    if (!platform::attachParentConsole()) {
        const auto logPath = platform::logsDir() / "viewer.log";
        FILE* f = nullptr;
        if (freopen_s(&f, logPath.string().c_str(), "w", stderr) == 0) freopen_s(&f, logPath.string().c_str(), "a", stdout);
    }
    ViewerOptions opt;
    if (const auto cfg = configPath(argc, argv); !cfg.empty()) {
        if (applyConfig(opt, cfg)) LOG_INFO("app") << "settings: " << cfg.string();
    }
    if (!parse(argc, argv, opt)) {
        usage(argv[0]);
        return 2;
    }
    if (opt.help) {
        if (opt.helpAll) usageAll(argv[0]);
        else usage(argv[0]);
        return 0;
    }
    if (opt.list) {
        const auto worlds = ipc::WorldRegistry::list();
        if (worlds.empty()) std::printf("no published worlds\n");
        for (const auto& w : worlds) std::printf("%-32s pid %llu\n", w.name.c_str(), static_cast<unsigned long long>(w.pid));
        return 0;
    }

    // --- Rendering (window first: the terrain ground provider uses VSG's readers) ---
    render::Viewer viewer;
    opt.window.title = opt.demo ? "flightsim - demo " + opt.aircraft
                      : !opt.replayPath.empty() ? "flightsim - replay " + opt.replayPath
                                                : "flightsim - waiting for a training application";
    if (opt.earth.elevationUrl.empty()) opt.earth.elevationUrl = world::kAwsTerrariumUrl;
    if (opt.earth.source == world::EarthSettings::Source::None) opt.earth.elevationUrl.clear();
    opt.window.headlight = !opt.sun;
    if (opt.tileCache.empty()) opt.tileCache = platform::mapsDir(); // (the local map set, never %LOCALAPPDATA%)
    opt.window.tileCacheDir = opt.tileCache; // the renderer and the CPU tiles share one cache
    if (opt.offline) {
        if (!std::filesystem::is_directory(opt.tileCache)) {
            LOG_ERROR("app") << "no map set at " << opt.tileCache.string() << ": run fetch-maps.cmd, or give --tile-cache";
            return 1;
        }
        opt.earth.offlineRoot = opt.tileCache; // layer URLs become file paths
        LOG_INFO("app") << "offline: tiles read from " << opt.tileCache.string() << ", nothing is fetched";
    }
    if (!viewer.create(opt.window)) return 1;

    io::AssetResolver assets;
    for (auto it = opt.assetDirs.rbegin(); it != opt.assetDirs.rend(); ++it) assets.addSearchPath(*it);
    auto ellipsoid = vsg::EllipsoidModel::create(); // WGS-84

    // Elevation tiles on the CPU: camera terrain collision in every mode, physics in demo mode.
    // (The same headless io::TerrainTiles the SDK uses; the download cache is shared with the renderer's.)
    std::shared_ptr<io::TerrainTiles> terrain;
    if (!opt.earth.elevationUrl.empty() && opt.earth.elevationEncoding == world::ElevationEncoding::Terrarium) {
        io::TerrainTiles::Options to;
        to.urlTemplate = opt.earth.elevationUrl;
        to.zoom = opt.terrainZoom;
        to.cacheDir = opt.tileCache;
        to.offline = opt.offline;
        terrain = std::make_shared<io::TerrainTiles>(to);
    }

    // --- Demo simulation (only with --demo) --------------------------------------
    std::unique_ptr<sim::SimRunner> runner;
    std::size_t slots = opt.capacity;

    // --- Recording playback (--replay) ------------------------------------------------
    ipc::Recording recording;
    if (!opt.replayPath.empty()) {
        std::string error;
        if (!recording.load(opt.replayPath, &error)) {
            LOG_ERROR("app") << error;
            return 1;
        }
        slots = std::max<std::size_t>(1, recording.header().capacity);
    }
    const bool replay = !recording.frames().empty();
    std::size_t replayFrame = static_cast<std::size_t>(-1);
    double replayTime = replay ? recording.frames().front().simTime : 0.0;
    if (opt.demo) {
        const auto root = assets.jsbsimRoot(opt.jsbsimRoot);
        if (!root) {
            LOG_ERROR("app") << "JSBSim data root not found; pass --jsbsim-root <dir>";
            return 1;
        }
        std::shared_ptr<sim::GroundProvider> ground;
        if (terrain) {
            terrain->prefetch(units::degreesToRadians(opt.latitudeDeg), units::degreesToRadians(opt.longitudeDeg), 1500.0 + opt.spreadDeg * 111000.0);
            ground = terrain;
        } else {
            ground = std::make_shared<sim::FlatGround>(0.0);
        }
        const unsigned physical = platform::physicalCoreCount();
        const unsigned workers = std::min<unsigned>(opt.workers ? opt.workers : std::max(1u, physical > 3 ? physical - 3 : 1u), opt.vehicles);
        auto pool = std::make_unique<sim::VehiclePool>(workers);
        const sim::AircraftSpec aircraft{opt.aircraft, *root, assets.findAircraft(opt.aircraft, *root)};
        auto initialConditions = std::make_shared<std::vector<sim::InitialConditions>>();
        for (unsigned i = 0; i < opt.vehicles; ++i) {
            Rng rng = Rng::forVehicle(opt.seed, 0, i);
            sim::InitialConditions ic;
            ic.latitudeDeg = opt.latitudeDeg + rng.uniform(-opt.spreadDeg, opt.spreadDeg);
            ic.longitudeDeg = opt.longitudeDeg + rng.uniform(-opt.spreadDeg, opt.spreadDeg);
            ic.headingDeg = rng.uniform(0.0, 360.0);
            if (opt.onGround) {
                ic.onGround = true;
                ic.airspeedTrueMs = 0.0;
            } else {
                const double terrainM = ground->heightAboveEllipsoidM(units::degreesToRadians(ic.latitudeDeg), units::degreesToRadians(ic.longitudeDeg));
                ic.altitudeMslM = std::max(opt.altitudeM + rng.uniform(-150.0, 150.0), terrainM + 300.0);
                ic.airspeedTrueMs = opt.speedMs > 0.0 ? opt.speedMs : 58.0 + rng.uniform(-4.0, 4.0);
            }
            auto model = std::make_unique<sim::JsbsimModel>(opt.dt, ground);
            if (!model->load(aircraft, ic)) {
                LOG_ERROR("app") << "vehicle " << i << " failed to load";
                return 1;
            }
            if (!opt.onGround && opt.speedMs <= 0.0) {
                // a fighter would start below its stall at 58 m/s: fly off where
                // the aircraft is comfortable, its weight carried at CL 0.5
                const double weightLbf = model->property("inertia/weight-lbs").get();
                const double areaFt2 = model->property("metrics/Sw-sqft").get();
                const double rhoSlugFt3 = model->property("atmosphere/rho-slugs_ft3").get();
                if (weightLbf > 0.0 && areaFt2 > 0.0 && rhoSlugFt3 > 0.0) {
                    const double v = 0.3048 * std::sqrt(2.0 * weightLbf / (areaFt2 * rhoSlugFt3 * 0.5));
                    if (v > ic.airspeedTrueMs) {
                        ic.airspeedTrueMs = v * (1.0 + rng.uniform(-0.05, 0.05));
                        model->reset(ic);
                    }
                }
            }
            pool->add(std::move(model));
            initialConditions->push_back(ic);
        }
        auto autopilot = std::make_shared<app::DemoAutopilot>(opt.vehicles, opt.throttle, opt.onGround);
        runner = std::make_unique<sim::SimRunner>(std::move(pool), opt.frameSkip,
            [autopilot, initialConditions](const sim::SnapshotBatch& prev, sim::VehiclePool& vehicles, std::vector<sim::ControlInputs>& out) {
                for (std::size_t i = 0; i < prev.states.size(); ++i) {
                    if (!prev.states[i].diverged) continue;
                    LOG_WARN("app") << "vehicle " << i << " diverged; resetting to its initial conditions";
                    vehicles.vehicle(i).reset((*initialConditions)[i]);
                    autopilot->forget(i);
                }
                autopilot->compute(prev.states, out);
            });
        runner->setTimeFactor(opt.timeFactor);
        slots = opt.vehicles;
        LOG_INFO("app") << "demo: " << opt.vehicles << " x " << opt.aircraft << " on " << workers << " worker(s)";
    }

    // --- Scene -------------------------------------------------------------------
    auto scene = vsg::Group::create();
    int day = 172; double hours = 12.0;
    world::currentUtc(day, hours);
    if (opt.sunUtcHours >= 0.0) hours = opt.sunUtcHours;
    vsg::dvec3 sunDir = world::sunDirectionEcef(day, hours);
    world::SkyDome sky(viewer.options(), sunDir);
    scene->addChild(sky.node());
    vsg::ref_ptr<vsg::Node> sunLight;
    if (opt.sun) {
        sunLight = world::createSunLight(day, hours);
        scene->addChild(sunLight);
    }
    // the sea's depth and where it is (GEBCO 2026, tools/ocean_map, from the local map set): R the depth, G the water's
    // share of the texel; without it the imagery's own sea
    vsg::ref_ptr<vsg::Data> seaMap;
    {
        const auto path = opt.tileCache / "ocean" / "gebco-ocean-2arcmin.png";
        std::vector<std::uint8_t> pixels;
        std::uint32_t w = 0, h = 0;
        if (io::readRgbaPng(path, pixels, w, h)) {
            auto map = vsg::ubvec2Array2D::create(w, h, vsg::Data::Properties{VK_FORMAT_R8G8_UNORM});
            auto* texels = map->data();
            for (std::size_t i = 0, n = std::size_t{w} * h; i < n; ++i) texels[i] = vsg::ubvec2(pixels[i * 4], pixels[i * 4 + 3]);
            seaMap = map;
            // (its floor drawn below sea level - below a storm's troughs - so the land at the shore hides it however low
            // it lies, and the sea floor never does)
            world::setSeaFloorDepth(-25.0f);
            LOG_INFO("app") << "the sea: " << path.string() << " (" << w << " x " << h << ")";
        } else {
            LOG_INFO("app") << "no sea map at " << path.string() << " (tools/ocean_map/make_ocean_map.py): the imagery's sea";
        }
    }
    std::unique_ptr<world::Atmosphere> atmosphere;
    if (auto earth = world::createEarth(opt.earth, viewer.options(), ellipsoid)) {
        scene->addChild(earth);
        if (auto caps = world::createPolarCaps(ellipsoid, viewer.options())) scene->addChild(caps); // Mercator tiles end at 85 deg
        // (added after the clouds' shade and the sea, so the air is blended over the globe rather than hidden by it)
        atmosphere = std::make_unique<world::Atmosphere>(viewer.options(), ellipsoid, sunDir);
    }
    // the cloud behind the vehicles (and its shadow on the ground) after the ground and the air, and before the
    // vehicles; the cloud in front of them just after them
    // takram's clouds (assets/clouds), and the Earth's cloud cover farther off (NASA's Blue Marble, tools/cloud_map,
    // from the local map set)
    vsg::ref_ptr<vsg::Data> earthClouds;
    {
        const auto path = opt.tileCache / "clouds" / "earth-clouds-8192.png";
        std::vector<std::uint8_t> pixels;
        std::uint32_t w = 0, h = 0;
        if (io::readGreyPng(path, pixels, w, h)) {
            auto map = vsg::ubyteArray2D::create(w, h, vsg::Data::Properties{VK_FORMAT_R8_UNORM});
            std::copy(pixels.begin(), pixels.end(), map->data());
            earthClouds = map;
            LOG_INFO("app") << "the Earth's clouds: " << path.string() << " (" << w << " x " << h << ")";
        } else {
            LOG_INFO("app") << "no Earth cloud map at " << path.string() << " (tools/cloud_map/make_cloud_map.py): the world's cover everywhere";
        }
    }
    const auto cloudFiles = assets.find("clouds");
    if (!cloudFiles) LOG_WARN("app") << "no clouds: share/flightsim/clouds is missing";
    world::Clouds clouds(viewer.options(), cloudFiles.value_or(std::filesystem::path{}), earthClouds, seaMap);
    std::optional<world::Clouds::Weather> ownSky; // a sky of our own (--clouds), else the world's
    {
        world::Clouds::Weather w;
        const std::string& c = opt.clouds;
        if (c == "clear") ownSky = w;
        else if (c == "few") { w.cover = 0.2; w.baseM = 1500.0; ownSky = w; }
        else if (c == "scattered") { w.cover = 0.4; w.baseM = 1500.0; ownSky = w; }
        else if (c == "broken") { w.cover = 0.7; w.baseM = 1200.0; ownSky = w; }
        else if (c == "overcast") { w.cover = 0.95; w.baseM = 900.0; ownSky = w; }
        else if (c == "rain") { w.cover = 1.0; w.baseM = 600.0; w.precipitation = 0.8; ownSky = w; }
        else if (c != "world" && c != "off") LOG_WARN("app") << "--clouds " << c << ": not a sky; the world's weather instead";
        if (ownSky) clouds.setWeather(*ownSky);
        if (opt.seaWindMs >= 0.0) clouds.setSeaWind(opt.seaWindMs);
    }
    scene->addChild(clouds.under()); // (the clouds' shade on the ground, and the sea)
    if (atmosphere) scene->addChild(atmosphere->node());

    world::VehicleVisuals::Settings visualSettings;
    if (opt.modelPath.empty()) {
        if (auto sample = assets.find("models/Cesium_Air.glb")) opt.modelPath = sample->string();
    } else if (opt.modelPath == "none") {
        opt.modelPath.clear();
    }
    visualSettings.modelPath = opt.modelPath;
    world::VehicleVisuals::applyManifest(visualSettings);
    if (opt.modelScaleOverride > 0.0) visualSettings.modelScale = opt.modelScaleOverride;
    auto axis = [](const std::string& a, const vsg::dvec3& fallback) {
        if (a.size() != 2) return fallback;
        const double sgn = a[0] == '-' ? -1.0 : 1.0;
        switch (a[1]) {
        case 'x': return vsg::dvec3(sgn, 0.0, 0.0);
        case 'y': return vsg::dvec3(0.0, sgn, 0.0);
        case 'z': return vsg::dvec3(0.0, 0.0, sgn);
        default: return fallback;
        }
    };
    visualSettings.modelForward = axis(opt.modelForward, visualSettings.modelForward);
    visualSettings.modelUp = axis(opt.modelUp, visualSettings.modelUp);
    for (const auto& dir : assets.searchPaths()) // <assets>/models/<type>.glb per vehicle type
        if (std::filesystem::is_directory(dir / "models")) visualSettings.modelDirs.push_back(dir / "models");
    for (const auto& dir : assets.aircraftDirs()) // aircraft of our own carry their model: <dir>/<type>/<type>.glb
        visualSettings.modelDirs.push_back(dir);
    world::VehicleVisuals visuals(slots, visualSettings, viewer.options());
    visuals.setCompiler([&viewer](vsg::ref_ptr<vsg::Node> node) { return viewer.compile(node); });
    scene->addChild(visuals.node());
    scene->addChild(clouds.over());
    world::Trails trails(slots, 900, 0.25, viewer.options());
    scene->addChild(trails.node());
    // After the vehicles: vapour and contrails blend over them.
    world::Airflow airflow(slots, viewer.options(), cloudFiles ? *cloudFiles / "shaders" : std::filesystem::path{}, clouds.atmosphere(),
                           viewer.coarseShading());
    std::unique_ptr<world::FlowVolumes> flow; // (the engagement's fires as a fluid: below)
    // each aircraft's span and mass without fuel, from its JSBSim definition (the contrails' wake vortex), by type
    std::map<std::string, std::optional<io::AircraftMetrics>> metricsByType;
    const auto describeAircraft = [&](std::size_t slot, std::string type) {
        if (type.rfind("jsbsim:", 0) == 0) type = type.substr(7);
        auto found = metricsByType.find(type);
        if (found == metricsByType.end()) {
            std::optional<io::AircraftMetrics> m;
            if (const auto root = assets.jsbsimRoot(opt.jsbsimRoot); root && !type.empty()) {
                const auto dir = assets.findAircraft(type, *root);
                m = io::readAircraftMetrics((dir.empty() ? *root / "aircraft" : dir) / type / (type + ".xml"));
            }
            found = metricsByType.emplace(type, m).first;
        }
        const auto& m = found->second;
        airflow.setAircraft(slot, m ? m->wingspanM : 0.0, m ? m->emptyMassKg + m->pointMassKg : 0.0);
        if (flow) flow->setAircraft(slot, m ? m->wingspanM : 0.0, m ? m->emptyMassKg + m->pointMassKg : 0.0);
    };
    scene->addChild(airflow.node());
    // the engagement's detonations (MS-7h): their fireballs and smoke, drawn as the contrails are
    world::Detonations detonations(viewer.options(), cloudFiles ? *cloudFiles / "shaders" : std::filesystem::path{}, clouds.atmosphere(),
                                   viewer.coarseShading());
    scene->addChild(detonations.node());
    // ... or solved as a fluid (NVIDIA Flow, when built with it): the fireballs then not drawn
    if (opt.fire == "flow") {
        flow = std::make_unique<world::FlowVolumes>(viewer.options(), cloudFiles ? *cloudFiles / "shaders" : std::filesystem::path{},
                                                    clouds.atmosphere(), viewer.coarseShading());
        if (flow->available()) {
            scene->addChild(flow->node());
            airflow.setFireHandOver(world::FlowVolumes::nearFieldThinning());
        }
        else flow.reset();
    }
    world::WingVapour vapour(slots, viewer.options());
    // (the gas round the aircraft is drawn over the scene: Viewer::setScene's overScene)
    if (!opt.demo)
        for (std::size_t i = 0; i < slots; ++i) { visuals.setVisible(i, false); trails.setEnabled(i, false); }

    auto controls = std::make_shared<ui::ViewerControls>();
    controls->selectedVehicle.store(opt.follow, std::memory_order_relaxed);
    bool followPending = opt.follow > 0; // (until that vehicle exists)
    // the stores on each vehicle's stations (MS-7): drawn while carried - loaded, powering up or down, ready - and gone
    // from the station as they are released or jettisoned (a released one is a vehicle of its own then)
    const auto carriedOf = [](const ipc::MissionStore* stores, std::size_t n) {
        std::vector<world::VehicleVisuals::Carried> out;
        for (std::size_t k = 0; k < n; ++k) {
            const auto status = static_cast<ms::StoreStatus>(stores[k].status);
            const std::string card(stores[k].store, ::strnlen(stores[k].store, sizeof stores[k].store));
            if (card.empty() || status == ms::StoreStatus::Releasing || status == ms::StoreStatus::Released ||
                status == ms::StoreStatus::Jettisoned)
                continue;
            out.push_back({stores[k].station, world::VehicleVisuals::storeType(card)});
        }
        return out;
    };
    // and the weapon bays' doors, station by station (every store's, those gone included: the doors shut after them)
    const auto doorsOf = [](const ipc::MissionStore* stores, std::size_t n) {
        std::vector<std::pair<int, double>> out;
        for (std::size_t k = 0; k < n; ++k) out.emplace_back(stores[k].station, stores[k].bayDoors / 255.0);
        return out;
    };
    auto storeRecord = std::make_unique<ipc::MissionRecord>();
    std::vector<std::uint64_t> storeRevision; // per slot: the mission record's revision last drawn (~0: read it again)
    if (!opt.gui) {
        // Looking at the graphics, not at the instruments: nothing drawn over
        // the scene, and nothing drawn in it that is not the world itself.
        controls->showMonitor.store(false);
        controls->showVehicleList.store(false);
        controls->showCameras.store(false);
        controls->showLabels.store(false);
        controls->showTrails.store(false);
    }
    controls->showEffects.store(opt.effects);
    if (opt.clouds == "off") controls->showClouds.store(false); // (key k shows them)
    controls->timeFactor.store(opt.timeFactor);
    controls->cameraMode.store(opt.cameraMode);
    auto gui = ui::MonitorGui::create(controls, replay ? "replay " + opt.replayPath : opt.aircraft);
    auto imgui = viewer.createImGui(); // (for the pass over the scene, where it is drawn last)
    imgui->addChild(gui);
    auto cameraPreview = ui::CameraPreview::create(controls, [&viewer](vsg::ref_ptr<vsg::Object> o) { return viewer.compile(o); },
                                                   viewer.window()->getOrCreateDevice()->deviceID);
    imgui->addChild(cameraPreview);
    auto missionGui = ui::MissionGui::create(controls); // (the followed vehicle's payloads and radar displays, from the world's records)
    imgui->addChild(missionGui);
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::GetIO().ConfigWindowsMoveFromTitleBarOnly = true; // a drag that starts inside a panel is never a panel move
    if (const double dpi = platform::systemDpiScale(); dpi > 1.01) {
        ImGui::GetIO().FontGlobalScale = static_cast<float>(dpi);
        ImGui::GetStyle().ScaleAllSizes(static_cast<float>(dpi));
    }
    viewer.addEventHandler(vsgImGui::SendEventsToImGui::create());
    viewer.addEventHandler(ui::KeyHandler::create(controls));
    viewer.setDepthPrepass(visuals.node()); // (where the vapour's rays end)
    // the gas round the aircraft, built before the scene is set: it is drawn in the pass over the scene, whose
    // pipelines are made when the scene is compiled
    vapour.build(viewer.depthPrepass().first, viewer.window()->extent2D().width, viewer.window()->extent2D().height, nullptr,
                 visuals.exhaustData(), cloudFiles ? world::loadBlueNoise(*cloudFiles) : vsg::ref_ptr<vsg::ImageInfo>{},
                 viewer.sceneColour());
    auto beforePass = vsg::Group::create();
    // --stats: the GPU's time per pass too (timestamps between them)
    render::GpuTimer gpuTimer;
    if (opt.stats > 0.0) {
        beforePass->addChild(gpuTimer.reset());
        beforePass->addChild(gpuTimer.mark("start"));
    }
    beforePass->addChild(clouds.compute());
    if (opt.stats > 0.0) beforePass->addChild(gpuTimer.mark("clouds march"));
    beforePass->addChild(vapour.compute());
    if (opt.stats > 0.0) {
        beforePass->addChild(gpuTimer.mark("vapour march"));
        const std::vector<std::pair<vsg::ref_ptr<vsg::Node>, std::string>> named{
            {clouds.under(), "clouds under"}, {visuals.node(), "vehicles+exhaust"}, {clouds.over(), "clouds over"},
            {trails.node(), "trails"}, {airflow.node(), "contrails"}};
        auto marked = vsg::Group::create();
        marked->addChild(gpuTimer.mark("prepass, pass begin"));
        for (const auto& child : scene->children) {
            marked->addChild(child);
            std::string label = "sky, earth, rest";
            for (const auto& [node, name] : named)
                if (node && node == child) label = name;
            marked->addChild(gpuTimer.mark(label));
        }
        scene = marked;
    }
    // the gas round the aircraft, drawn over the scene (the scene's colour copied for its heat haze first)
    vsg::ref_ptr<vsg::Node> over = vapour.node();
    if (opt.stats > 0.0) {
        auto marked = vsg::Group::create();
        marked->addChild(over);
        marked->addChild(gpuTimer.mark("pass end, scene copy, gas"));
        over = marked;
    }
    if (!viewer.setScene(scene, ellipsoid, imgui, beforePass, over)) return 1;
    vapour.compiled();
    clouds.build(viewer.window()->extent2D().width, viewer.window()->extent2D().height,
                 [&viewer](vsg::ref_ptr<vsg::Node> node) { return viewer.compile(node); });
    if (opt.demo) // the demo's vehicles are all of one type: draw that type's model if it has one (after the scene compile)
        for (unsigned i = 0; i < opt.vehicles && i < slots; ++i) {
            visuals.setModel(i, {}, "jsbsim:" + opt.aircraft);
            describeAircraft(i, opt.aircraft);
        }
    for (auto& anim : visuals.animations()) viewer.viewer()->animationManager->play(anim);

    auto camera = world::CameraController::create(viewer.camera(), viewer.lookAt(), ellipsoid);
    camera->setChaseOffset(opt.chaseDistance, opt.chaseElevation, opt.chaseAzimuth);
    camera->setZoomToCursor(opt.zoomToCursor);
    // The camera must be told the same ground the vertex shader displaces the
    // mesh with, or it clears a hill that is not the one on screen. VSG applies
    // elevation as a displacement map, so there is no CPU geometry to intersect
    // the way osgEarth does - the only way for the two to agree is to read the
    // same tiles at the same level. Asking one level coarser than the renderer
    // (z14 against z15) smoothed the peaks away: measured over the Sierra the
    // camera under-read the drawn ground by 14 m, and the line of sight passed
    // 2-4 m under the surface at Yosemite, Everest and the Bernese Alps while
    // believing itself clear. Hence elevationMaxLevel here, not a number.
    std::shared_ptr<io::TerrainTiles> cameraGround;
    if (terrain) {
        io::TerrainTiles::Options to;
        to.urlTemplate = opt.earth.elevationUrl;
        to.zoom = opt.earth.elevationMaxLevel;
        // Resampled to 64 a tile is 16 KB rather than 256, so the working set
        // is cheap: 512 of them is ~8 MB, less than the 64 raw tiles cost
        // before, and covers a few hundred km of ground.
        to.cacheTiles = 512;
        to.cacheDir = opt.tileCache;
        to.offline = opt.offline;
        // Sample the surface as the mesh is built, not the raw raster.
        to.meshDimension = opt.earth.elevationMeshDimension;
        cameraGround = std::make_shared<io::TerrainTiles>(to);
        camera->setGroundQuery([cameraGround](double lat, double lon) { return cameraGround->cachedHeightAboveEllipsoidM(lat, lon); });
        // the sea's shore round the eye, read on the clouds' worker thread from tiles of its own (the raster as it is:
        // where the coast is, not where the mesh draws it)
        io::TerrainTiles::Options so = to;
        so.meshDimension = 0;
        so.cacheTiles = 64;
        auto shoreGround = std::make_shared<io::TerrainTiles>(so);
        clouds.setShoreQuery([shoreGround](double lat, double lon) { return shoreGround->knownHeightAboveEllipsoidM(lat, lon); });
    }
    // Detached camera focus: the demo spawn area / default location, on the ground.
    camera->setFocus(ellipsoid->convertLatLongAltitudeToECEF(vsg::dvec3(opt.latitudeDeg, opt.longitudeDeg, 0.0)));
    // Looking down at the ground, not across it - unless the command line
    // asked for an angle: the camera is detached until the first vehicle
    // appears, and it keeps the elevation it had then.
    camera->setDetachedElevation(opt.chaseElevationSet ? opt.chaseElevation : 45.0);
    if (!opt.demo && !opt.chaseDistanceSet) camera->zoom(300.0); // no vehicle yet: start with a regional view
    if (!opt.view.empty()) {
        double v[6] = {0.0, 0.0, 0.0, 1000.0, 180.0, 45.0};
        std::size_t start = 0;
        for (int i = 0; i < 6 && start <= opt.view.size(); ++i) {
            const auto comma = opt.view.find(',', start);
            v[i] = std::atof(opt.view.substr(start, comma == std::string::npos ? std::string::npos : comma - start).c_str());
            if (comma == std::string::npos) break;
            start = comma + 1;
        }
        camera->setFreeView(v[0], v[1], v[2], v[3], v[4], v[5]);
    }
    viewer.addEventHandler(camera);

    // --- Source state shared by both modes -----------------------------------------
    world::Interpolator interpolator(slots);
    // An engagement's event as it is drawn - a replay's from its recording, a live world's from its segment's ring:
    // a detonation's and a spill's fireball, a fire burning on from its component, a section that left hidden on its
    // aircraft (its piece flies in a slot of its own)
    const auto showEvent = [&](const ipc::EngagementEvent& e) {
        if (flow) flow->add(e, *ellipsoid);
        else detonations.add(e, *ellipsoid); // (a fire's spill: its fireball)
        // (with Flow, its flame and near smoke burn there and the trail is the far smoke only; else the fire trail, its
        // flame and its mist)
        if (e.kind == static_cast<std::uint8_t>(ipc::EngagementKind::Fire))
            airflow.ignite(e.slot, std::string(e.name, ::strnlen(e.name, sizeof e.name)), e.rateKgS, e.simTime, e.massKg - e.rateKgS); // (what does not burn: its mist)
        if (e.kind == static_cast<std::uint8_t>(ipc::EngagementKind::Separated))
            visuals.hideSection(e.slot, std::string(e.name, ::strnlen(e.name, sizeof e.name)));
    };
    const auto forgetEvents = [&] {
        detonations.clear(), airflow.clearFires(), visuals.clearSections();
        if (flow) flow->clear();
    };
    std::vector<ipc::EngagementEvent> liveEvents;
    std::vector<std::uint64_t> generationSeen; // (each slot's vehicle as the table last had it)
    // what each slot's engines burn, as the simulation says it with each sample: the trails are laid from it
    std::vector<std::array<double, sim::VehicleState::kMaxEngines>> fuelFlows(slots);
    const auto fuelFlowOf = [&](std::size_t slot, const double* derived) {
        if (slot < fuelFlows.size())
            for (std::size_t k = 0; k < fuelFlows[slot].size(); ++k) fuelFlows[slot][k] = derived[ipc::kDerivedFuelFlowKgS + k];
    };
    std::vector<unsigned char> alive(slots, opt.demo ? 1 : 0);
    std::vector<ui::MonitorGui::VehicleMeta> meta;
    const sim::SnapshotBatch* batch = nullptr;  // latest raw batch (demo: runner's; mirror: `mirrored`)
    sim::SnapshotBatch mirrored, interpBatch;    // mirror copy; interpolation copy keyed by wall time

    ipc::WorldMirror mirror;
    double lastDiscovery = -10.0, lastSunUpdate = -1e9;
    std::uint32_t focusSeen = 0; // the last request to follow a vehicle the attached world made that was followed
    bool closeIn = false;        // a request followed from far out: the chase view, once the camera has its target
    std::vector<std::string> available;
    std::uint64_t lastVehicleSteps = 0;
    double lastStepsWall = 0.0, publishedSps = 0.0;

    if (runner) {
        runner->start();
        if (const auto* first = runner->snapshots().acquire()) {
            batch = first;
            interpolator.push(*first);
            visuals.update(Span<const sim::VehicleState>(first->states));
            camera->setTargetRadius(visuals.shape(0).reach());
            camera->update(&first->states[0], 0.0);
        }
    }

    // --- Frame loop ------------------------------------------------------------------
    double wallSeconds = 0.0;
    int shots = 0; // frames saved (--screenshot-frames)
    const bool shotsOnReplay = replay && opt.screenshotReplayClock && !opt.screenshot.empty();
    struct Stats {
        double window = 0.0, cpu0 = platform::processCpuSeconds(), maxFrame = 0.0;
        int frames = 0, late = 0;
        double lastMean = 0.0;
        render::Viewer::FrameTiming sum, worst;
        // NVIDIA Flow's (world::FlowVolumes::Cost)
        double flowGpu = 0.0, flowCpu = 0.0, flowPack = 0.0, flowGpuMax = 0.0, flowCpuMax = 0.0;
        std::uint64_t flowLeaves = 0, flowPages = 0;
        int flowFrames = 0;
    } stats;
    // the world's environment as the scene draws it - its clouds (unless --clouds gave a sky of its own), the air the
    // contrails, smoke and vapour form in, and the sun by its clock (unless --sun-utc): a live world's every frame, a
    // replay's as its recording has it (format 4; an older recording none: the defaults)
    std::optional<sim::EnvironmentState> replayEnvironment;
    const auto applyEnvironment = [&](const sim::EnvironmentState& env) {
        if (!ownSky) {
            world::Clouds::Weather w;
            w.cover = std::isfinite(env.cloudCover) ? env.cloudCover : 0.0;
            w.baseM = std::isfinite(env.cloudBaseM) ? env.cloudBaseM : 2000.0;
            w.precipitation = std::isfinite(env.precipitation) ? env.precipitation : 0.0;
            w.windFromDeg = std::isfinite(env.windDirectionDeg) ? env.windDirectionDeg : 0.0;
            w.windMs = std::isfinite(env.windSpeedMs) ? env.windSpeedMs : 0.0;
            clouds.setWeather(w);
        }
        {
            // the air the contrails form and spread in
            world::Airflow::Air a;
            if (std::isfinite(env.temperatureSeaLevelK) && env.temperatureSeaLevelK > 150.0) a.seaLevelK = env.temperatureSeaLevelK;
            if (std::isfinite(env.pressureSeaLevelPa) && env.pressureSeaLevelPa > 1000.0) a.seaLevelPa = env.pressureSeaLevelPa;
            a.humidity = std::isfinite(env.humidity) ? env.humidity : 0.0;
            a.windFromDeg = std::isfinite(env.windDirectionDeg) ? env.windDirectionDeg : 0.0;
            a.windMs = std::isfinite(env.windSpeedMs) ? env.windSpeedMs : 0.0;
            airflow.setAir(a);
            detonations.setAir(world::Detonations::Air{a.seaLevelK, a.seaLevelPa, a.windFromDeg, a.windMs});
            if (flow) flow->setAir(world::Detonations::Air{a.seaLevelK, a.seaLevelPa, a.windFromDeg, a.windMs});
        }
        if (std::isfinite(env.temperatureSeaLevelK) && env.temperatureSeaLevelK > 150.0)
            vapour.setAir(env.temperatureSeaLevelK, env.humidity); // what the air round the aircraft condenses
        if (opt.sunUtcHours < 0.0 && env.epochUtcSeconds > 0.0 && wallSeconds - lastSunUpdate > 1.0 && batch) {
            lastSunUpdate = wallSeconds;
            world::utcOf(env.epochUtcSeconds + batch->simTime, day, hours);
            sunDir = world::sunDirectionEcef(day, hours);
            sky.setSun(sunDir);
            if (atmosphere) atmosphere->setSun(sunDir);
        }
    };
    while (viewer.active() && !controls->quit.load(std::memory_order_relaxed)) {
        wallSeconds += viewer.frameSeconds();
        bool paused = false;
        double timeFactor = 1.0;

        if (runner) {
            paused = controls->paused.load(std::memory_order_relaxed);
            timeFactor = controls->timeFactor.load(std::memory_order_relaxed);
            runner->setPaused(paused);
            runner->setTimeFactor(timeFactor);
            if (controls->singleStep.exchange(false)) runner->singleStep();
            if (const auto* fresh = runner->snapshots().acquire()) {
                batch = fresh;
                interpolator.push(*fresh);
            }
            controls->simThroughput.store(runner->throughput(), std::memory_order_relaxed);
            if (batch) controls->simTime.store(batch->simTime, std::memory_order_relaxed);
        } else if (replay) {
            // Playback clock in simulation time, paced by the demo controls; loops at the end.
            paused = controls->paused.load(std::memory_order_relaxed);
            timeFactor = controls->timeFactor.load(std::memory_order_relaxed);
            const auto& frames = recording.frames();
            const bool step = controls->singleStep.exchange(false);
            if (!paused) {
                const double advanced = replayTime + viewer.frameSeconds() * timeFactor;
                // a screenshot on the replay's clock: this frame stops at the next shot's time, not past it
                const double shot = opt.screenshotAfter + shots * opt.screenshotEvery;
                replayTime = shotsOnReplay && replayTime < shot && advanced > shot ? shot : advanced;
            }
            if (step && replayFrame + 1 < frames.size()) replayTime = frames[replayFrame + 1].simTime;
            const double seek = controls->seekTo.exchange(-1.0);
            if (seek >= 0.0) {
                // Seeking: rebuild the table from the first frame so creations/removals stay consistent.
                replayTime = std::clamp(seek, frames.front().simTime, frames.back().simTime);
                replayFrame = static_cast<std::size_t>(-1);
                interpolator = world::Interpolator(slots);
                std::fill(alive.begin(), alive.end(), 0);
                meta.assign(slots, ui::MonitorGui::VehicleMeta{});
                for (auto& m : meta) m.alive = false;
                for (std::size_t i = 0; i < slots; ++i) { visuals.setVisible(i, false); trails.setEnabled(i, false); }
            }
            if (replayTime > frames.back().simTime) {
                replayTime = frames.front().simTime; // loop
                replayFrame = static_cast<std::size_t>(-1);
                interpolator = world::Interpolator(slots);
            }
            std::size_t target = replayFrame == static_cast<std::size_t>(-1) ? 0 : replayFrame;
            while (target + 1 < frames.size() && frames[target + 1].simTime <= replayTime) ++target;
            if (target != replayFrame) {
                // Apply every frame we skipped (table changes must not be lost) but only build the last one.
                const std::size_t from = replayFrame == static_cast<std::size_t>(-1) ? 0 : replayFrame + 1;
                if (meta.size() != slots) {
                    meta.assign(slots, ui::MonitorGui::VehicleMeta{});
                    for (auto& m : meta) m.alive = false; // only recorded rows are vehicles
                }
                for (std::size_t f = from; f <= target; ++f)
                    for (const auto& [slot, row] : frames[f].tableChanges) {
                        if (slot >= slots) continue;
                        alive[slot] = row.alive;
                        meta[slot].alive = row.alive != 0;
                        meta[slot].name.assign(row.name, ::strnlen(row.name, ipc::kNameLength));
                        meta[slot].type.assign(row.type, ::strnlen(row.type, ipc::kTypeLength));
                        meta[slot].level = control::levelName(static_cast<control::Level>(row.controlLevel));
                        if (row.alive) {
                            visuals.setModel(slot, std::string(row.model, ::strnlen(row.model, ipc::kPathLength)), meta[slot].type);
                            describeAircraft(slot, meta[slot].type);
                        }
                        visuals.setVisible(slot, row.alive != 0);
                        trails.setEnabled(slot, row.alive != 0);
                    }
                if (from == 0) replayEnvironment.reset(); // (from the start again: a seek, a loop)
                if (from == 0) forgetEvents();
                for (std::size_t f = from; f <= target; ++f)
                    for (const auto& e : frames[f].events) showEvent(e);
                for (std::size_t f = from; f <= target; ++f)
                    if (frames[f].environment) replayEnvironment = frames[f].environment;
                for (std::size_t f = from; f <= target; ++f)
                    for (const auto& [slot, stores] : frames[f].storeChanges)
                        if (slot < slots) {
                            visuals.setStores(slot, carriedOf(stores.data(), stores.size()));
                            visuals.setBayDoors(slot, doorsOf(stores.data(), stores.size()));
                        }
                gui->setVehicles(meta);
                replayFrame = target;
                const auto& frame = frames[target];
                if (mirrored.states.size() != slots) mirrored.states.resize(slots);
                for (auto& f : fuelFlows) f.fill(0.0);
                for (const auto& [slot, sample] : frame.samples)
                    if (slot < slots) {
                        mirrored.states[slot] = drawnState(sample.state, sample.derived);
                        fuelFlowOf(slot, sample.derived);
                    }
                mirrored.simTime = frame.simTime;
                mirrored.wallNs = 0; // interpolate in simulation time, paced by the time factor
                ++mirrored.sequence;
                batch = &mirrored;
                interpolator.push(mirrored);
                if (shotsOnReplay && target + 1 < frames.size()) {
                    // shots on the replay's clock draw at its time exactly: the frame after it, pushed ahead, brackets it
                    static sim::SnapshotBatch ahead;
                    ahead = mirrored;
                    for (const auto& [slot, sample] : frames[target + 1].samples)
                        if (slot < slots) ahead.states[slot] = drawnState(sample.state, sample.derived);
                    ahead.simTime = frames[target + 1].simTime;
                    interpolator.push(ahead);
                }
            }
            if (replayEnvironment) applyEnvironment(*replayEnvironment);
            controls->simTime.store(replayTime, std::memory_order_relaxed);
            controls->simThroughput.store(0.0, std::memory_order_relaxed);
            controls->replayStart.store(frames.front().simTime, std::memory_order_relaxed);
            controls->replayEnd.store(frames.back().simTime, std::memory_order_relaxed);
        } else {
            // The GUI asked for another world: drop this one; discovery below attaches to it.
            const int attachIndex = controls->attachWorld.exchange(-1);
            if (attachIndex >= 0 && static_cast<std::size_t>(attachIndex) < available.size() && available[static_cast<std::size_t>(attachIndex)] != mirror.name()) {
                opt.worldName = available[static_cast<std::size_t>(attachIndex)];
                if (mirror.valid()) {
                    LOG_INFO("app") << "leaving world '" << mirror.name() << "' for '" << opt.worldName << "'";
                    mirror.close();
                    cameraPreview->detach();
                    batch = nullptr;
                    std::fill(alive.begin(), alive.end(), 0);
                    for (std::size_t i = 0; i < slots; ++i) { visuals.setVisible(i, false); trails.setEnabled(i, false); }
                    meta.clear();
                    gui->setVehicles(meta);
                }
                lastDiscovery = -1.0;
            }
            // Discovery: attach to the requested (or newest) world; re-attach after a restart.
            // While attached the registry is still polled (every 2 s) so the GUI can offer the other worlds.
            if (mirror.valid() && wallSeconds - lastDiscovery > 2.0) {
                lastDiscovery = wallSeconds;
                available.clear();
                for (const auto& w : ipc::WorldRegistry::list()) available.push_back(w.name);
            }
            if (!mirror.valid() && wallSeconds - lastDiscovery > 0.5) {
                lastDiscovery = wallSeconds;
                const auto worlds = ipc::WorldRegistry::list();
                available.clear();
                for (const auto& w : worlds) available.push_back(w.name);
                std::string pick;
                if (!opt.worldName.empty()) {
                    for (const auto& w : worlds) if (w.name == opt.worldName) pick = w.name;
                } else if (!worlds.empty()) {
                    pick = worlds.front().name;
                }
                if (!pick.empty() && mirror.open(pick)) {
                    if (mirror.capacity() != slots) LOG_WARN("app") << "world capacity " << mirror.capacity() << " differs from --capacity " << slots
                                                                     << "; only the first " << std::min<std::size_t>(slots, mirror.capacity()) << " slots are shown";
                    LOG_INFO("app") << "attached to world '" << pick << "'";
                    focusSeen = 0; // (its last request that viewers follow a vehicle, if any, is followed on attaching)
                    forgetEvents(); // (another world's fires are not this one's)
                    for (auto& f : fuelFlows) f.fill(0.0);
                    cameraPreview->attach(pick);
                    interpolator = world::Interpolator(slots);
                    for (std::size_t i = 0; i < slots; ++i) trails.setEnabled(i, false);
                    lastVehicleSteps = mirror.vehicleSteps();
                    lastStepsWall = wallSeconds;
                }
            }
            if (mirror.valid()) {
                if (mirror.pollTable()) {
                    meta.assign(slots, ui::MonitorGui::VehicleMeta{});
                    const auto& table = mirror.vehicles();
                    for (std::size_t i = 0; i < slots && i < table.size(); ++i) {
                        const auto& v = table[i];
                        alive[i] = v.alive ? 1 : 0;
                        meta[i].name = v.name;
                        meta[i].type = v.type;
                        meta[i].alive = v.alive;
                        meta[i].level = control::levelName(static_cast<control::Level>(v.controlLevel));
                        if (v.alive) {
                            visuals.setModel(i, v.model, v.type);
                            describeAircraft(i, v.type);
                        }
                        // (made, reset or removed since it was last seen: whole again - what broke off it before is back)
                        if (generationSeen.size() != slots) generationSeen.assign(slots, ~0ull);
                        if (generationSeen[i] != v.generation) {
                            generationSeen[i] = v.generation;
                            visuals.restoreSections(i);
                        }
                        visuals.setVisible(i, v.alive);
                        trails.setEnabled(i, v.alive);
                        if (i < storeRevision.size()) storeRevision[i] = ~0ull;
                    }
                    gui->setVehicles(meta);
                }
                // the stores on the vehicles' stations, where their records changed
                if (storeRevision.size() != slots) storeRevision.assign(slots, ~0ull);
                for (std::size_t i = 0; i < slots; ++i) {
                    if (!alive[i] || !mirror.mission(static_cast<std::uint32_t>(i), *storeRecord, storeRevision[i])) continue;
                    storeRevision[i] = storeRecord->revision;
                    const std::size_t count = std::min<std::size_t>(storeRecord->storeCount, ipc::kMissionStores);
                    visuals.setStores(i, carriedOf(storeRecord->stores, count));
                    visuals.setBayDoors(i, doorsOf(storeRecord->stores, count));
                }
                // The world's process asked viewers to follow a vehicle (World::focusViewer): followed once, as a tab
                // to it would be; the user's own choice stands after it. Not in the table yet: looked for again next frame.
                if (const std::uint32_t seq = mirror.focusSequence(); seq != focusSeen) {
                    const std::uint32_t id = mirror.focusVehicle();
                    const auto& table = mirror.vehicles();
                    for (std::size_t i = 0; i < slots && i < table.size(); ++i) {
                        if (!table[i].alive || table[i].id != id) continue;
                        controls->selectedVehicle.store(static_cast<int>(i), std::memory_order_relaxed);
                        if (controls->cameraMode.load(std::memory_order_relaxed) == 3) // (free: follow it)
                            controls->cameraMode.store(0, std::memory_order_relaxed);
                        // from the regional view a viewer starts in (or anything as far out), the aircraft would be a
                        // speck: the camera comes in to its chase view; nearer, the user's distance stands
                        closeIn = std::max(camera->distance(), camera->targetDistance()) > 20.0 * opt.chaseDistance;
                        focusSeen = seq;
                        break;
                    }
                }
                if (mirror.pollSnapshot(mirrored)) {
                    if (mirrored.states.size() != slots) mirrored.states.resize(slots);
                    // each picture hung from its model's origin, where the simulation has its airframe (drawnState)
                    for (std::size_t i = 0; i < mirrored.states.size() && i < mirror.status().size(); ++i) {
                        mirrored.states[i] = drawnState(mirrored.states[i], mirror.status()[i].derived);
                        fuelFlowOf(i, mirror.status()[i].derived);
                    }
                    // the engagement's events since the last look: its detonations, its fires, what broke off
                    liveEvents.clear();
                    mirror.pollEvents(liveEvents);
                    for (const auto& e : liveEvents) showEvent(e);
                    batch = &mirrored;
                    interpBatch = mirrored;
                    interpBatch.simTime = static_cast<double>(mirrored.wallNs) * 1e-9; // interpolate in wall time: the trainer's pace is arbitrary
                    interpolator.push(interpBatch);
                }
                if (wallSeconds - lastStepsWall >= 1.0) {
                    const std::uint64_t steps = mirror.vehicleSteps();
                    publishedSps = static_cast<double>(steps - lastVehicleSteps) / (wallSeconds - lastStepsWall);
                    lastVehicleSteps = steps;
                    lastStepsWall = wallSeconds;
                }
                // its weather, its air and the sun by its clock
                applyEnvironment(mirror.environment());
                // Gone - or replaced by another world of the same name (a testbench session restarted): the new
                // world counts from 0 again, so everything mirrored from the old one is dropped and the new one
                // attached at once, its vehicles, focus and trails from scratch.
                const bool replaced = mirror.replaced();
                if (replaced || (!mirror.publisherAlive() && mirror.ageSeconds() > 5.0)) {
                    if (replaced) {
                        LOG_INFO("app") << "world '" << mirror.name() << "' was replaced; attaching to the new one";
                        lastDiscovery = -10.0;
                    } else {
                        LOG_INFO("app") << "world '" << mirror.name() << "' has gone; waiting for another";
                    }
                    mirror.close();
                    cameraPreview->detach();
                    batch = nullptr;
                    std::fill(alive.begin(), alive.end(), 0);
                    for (std::size_t i = 0; i < slots; ++i) { visuals.setVisible(i, false); trails.setEnabled(i, false); }
                    meta.clear();
                    gui->setVehicles(meta);
                }
            }
            ui::MonitorGui::Source src;
            src.mirror = true;
            src.attached = mirror.valid();
            src.world = mirror.name();
            src.ageSeconds = mirror.ageSeconds();
            src.publisherAlive = mirror.publisherAlive();
            src.simTime = batch ? batch->simTime : 0.0;
            src.vehicleStepsPerSecond = publishedSps;
            src.available = available;
            if (mirror.valid()) src.environment = describe(mirror.environment(), src.simTime);
            gui->setSource(src);
            if (batch) controls->simTime.store(batch->simTime, std::memory_order_relaxed);
        }

        // Selection: only live slots.
        int selected = controls->selectedVehicle.load(std::memory_order_relaxed);
        // --follow's vehicle, once it exists (a missile is one only from its launch): taken once, unless the user
        // has moved on with tab before it came
        if (followPending && static_cast<std::size_t>(opt.follow) < slots && alive[static_cast<std::size_t>(opt.follow)]) {
            selected = opt.follow;
            followPending = false;
        }
        if (const int step = controls->selectStep.exchange(0); step != 0) {
            followPending = false;
            const int n = static_cast<int>(slots);
            for (int k = 1; k <= n; ++k) {
                const int candidate = ((selected + step * k) % n + n) % n;
                if (alive[static_cast<std::size_t>(candidate)]) { selected = candidate; break; }
            }
        }
        if (selected < 0 || static_cast<std::size_t>(selected) >= slots || !alive[static_cast<std::size_t>(selected)]) {
            for (std::size_t i = 0; i < slots; ++i)
                if (alive[i]) { selected = static_cast<int>(i); break; }
        }
        controls->selectedVehicle.store(selected, std::memory_order_relaxed);

        // Sim -> scene
        const bool anyAlive = std::any_of(alive.begin(), alive.end(), [](unsigned char a) { return a != 0; });
        if (batch) {
            if (shotsOnReplay) interpolator.updateAt(replayTime);
            else interpolator.update(viewer.frameSeconds(), timeFactor, paused);
            const auto& states = opt.interpolate ? interpolator.states() : batch->states;
            visuals.setExhaust(controls->showEffects.load(std::memory_order_relaxed));
            visuals.update(Span<const sim::VehicleState>(states));
            viewer.setSceneCopied(!visuals.flames().empty()); // (the exhausts' heat haze reads it; nothing else does)
            visuals.setSelected(selected);
            trails.setSelected(selected);
            trails.setVisible(controls->showTrails.load(std::memory_order_relaxed));
            trails.update(Span<const sim::VehicleState>(batch->states), (runner || replay) ? batch->simTime : static_cast<double>(batch->wallNs) * 1e-9);

            controls->snapshotSequence.store(batch->sequence, std::memory_order_relaxed);
        }
        camera->setMode(static_cast<world::CameraController::Mode>(controls->cameraMode.load(std::memory_order_relaxed)));
        if (const double z = controls->cameraZoom.exchange(1.0); z != 1.0) camera->zoom(z);
        if (controls->cameraReset.exchange(false)) camera->resetView();
        const sim::VehicleState* target = (batch && anyAlive && selected >= 0)
            ? &(opt.interpolate ? interpolator.states() : batch->states)[static_cast<std::size_t>(selected)] : nullptr;
        camera->setTargetRadius(target ? visuals.shape(static_cast<std::size_t>(selected)).reach() : 0.0);
        camera->update(target, viewer.frameSeconds());
        if (closeIn && target) {
            camera->resetView();
            closeIn = false;
        }
        if (batch) {
            // the labels, where the vehicles are drawn: with this frame's camera, not the last one's (put on the screen
            // before the camera moved, they were as far off as it had moved since - a fighter's chase camera 1.5 m a
            // frame at 160 fps, more after a long frame - and jumped about their aircraft as the frames' lengths varied)
            gui->setShowLabels(controls->showLabels.load(std::memory_order_relaxed));
            const auto& states = opt.interpolate ? interpolator.states() : batch->states;
            std::vector<ui::MonitorGui::Label> labels;
            const vsg::dmat4 viewProj = viewer.camera()->projectionMatrix->transform() * viewer.camera()->viewMatrix->transform();
            const auto extent = viewer.window()->extent2D();
            for (std::size_t i = 0; i < states.size(); ++i) {
                if (!alive[i]) continue;
                const vsg::dvec4 clip = viewProj * vsg::dvec4(states[i].positionEcef[0], states[i].positionEcef[1], states[i].positionEcef[2], 1.0);
                if (clip.w <= 0.0) continue;
                const double nx = clip.x / clip.w, ny = clip.y / clip.w;
                if (nx < -1.1 || nx > 1.1 || ny < -1.1 || ny > 1.1) continue;
                ui::MonitorGui::Label l;
                l.x = static_cast<float>((nx * 0.5 + 0.5) * extent.width);
                l.y = static_cast<float>((ny * 0.5 + 0.5) * extent.height);
                l.text = (i < meta.size() && !meta[i].name.empty() ? meta[i].name : "v" + std::to_string(i)) + (states[i].diverged ? " !" : "");
                l.selected = static_cast<int>(i) == selected;
                // (what its Vehicle Interface says of it, in the air: low on energy, deviating, upset - on its label)
                if (mirror.valid() && i < mirror.status().size() && !states[i].onGround) {
                    const char* word = "";
                    l.severity = ui::MonitorGui::severity(mirror.status()[i], &word);
                    if (l.severity > 0) l.text += "  ", l.text += word;
                }
                labels.push_back(std::move(l));
            }
            gui->setLabels(std::move(labels));
        }
        sky.update(viewer.lookAt()->eye);
        // the clouds, before the light: a deck overhead dims the sun where the camera looks
        clouds.setVisible(controls->showClouds.load(std::memory_order_relaxed));
        clouds.setScreen(viewer.window()->extent2D().width, viewer.window()->extent2D().height);
        // (the ground the clouds' shadow falls on: under what the camera looks at - the followed vehicle, or the
        // free camera's focus - and not under the selected vehicle wherever the camera is)
        double groundM = target ? target->altitudeMslM - target->altitudeAglM : 0.0;
        if (cameraGround) {
            const vsg::dvec3 looked = ellipsoid->convertECEFToLatLongAltitude(viewer.lookAt()->center);
            if (const auto h = cameraGround->cachedHeightAboveEllipsoidM(vsg::radians(looked.x), vsg::radians(looked.y)))
                groundM = *h;
        }
        clouds.update(viewer.camera()->viewMatrix->transform(), viewer.camera()->projectionMatrix->transform(), viewer.lookAt()->eye,
                      *ellipsoid, sunDir, groundM, wallSeconds);
        const double overcast =
            clouds.shown() ? world::Clouds::overcast(clouds.weather(), ellipsoid->convertECEFToLatLongAltitude(viewer.lookAt()->center).z) : 0.0;
        // the light for where the camera looks: the sun's elevation there sets the sun, the sky's light and the haze
        if (sunLight) world::setDaylight(sunLight, sunDir, viewer.lookAt()->center, overcast);
        if (atmosphere) atmosphere->update(viewer.lookAt()->eye);
        {
            // How much day there is where the camera is (the sky dome's own
            // measure): a flame is all light at night and part of the view by
            // day, and vapour white in the sun and grey without it.
            const double sunUp = vsg::dot(vsg::normalize(sunDir), vsg::normalize(viewer.lookAt()->eye));
            const float light = static_cast<float>(std::clamp((sunUp + 0.10) / 0.25, 0.0, 1.0));
            visuals.setDaylight(light);
            airflow.setVisible(controls->showEffects.load(std::memory_order_relaxed));
            airflow.setScreen(viewer.window()->extent2D().width, viewer.window()->extent2D().height);
            detonations.setVisible(controls->showEffects.load(std::memory_order_relaxed));
            detonations.setScreen(viewer.window()->extent2D().width, viewer.window()->extent2D().height);
            if (flow) {
                flow->setVisible(controls->showEffects.load(std::memory_order_relaxed));
                flow->setScreen(viewer.window()->extent2D().width, viewer.window()->extent2D().height);
            }
            vapour.setVisible(controls->showEffects.load(std::memory_order_relaxed));
            vapour.setScreen(viewer.window()->extent2D().width, viewer.window()->extent2D().height);
            if (batch) {
                const Span<const sim::VehicleState> drawn(opt.interpolate ? interpolator.states() : batch->states);
                // (the world's clock as drawn: a vehicle's, else the snapshot's)
                // (the snapshot's time, less how far behind it the drawn states are - as a vehicle that still moves
                // shows it: one that has struck the ground is stopped, its own clock with it, and an engagement's
                // fire drawn by that clock never grew)
                double drawnTime = batch->simTime;
                for (std::size_t i = 0; i < drawn.size() && i < alive.size() && i < batch->states.size(); ++i)
                    if (alive[i] && !drawn[i].crashed && !batch->states[i].crashed && std::isfinite(drawn[i].simTime) &&
                        std::isfinite(batch->states[i].simTime)) {
                        drawnTime = batch->simTime + std::min(drawn[i].simTime - batch->states[i].simTime, 0.0);
                        break;
                    }
                airflow.setFuelFlows(fuelFlows);
                airflow.setClock(drawnTime);
                airflow.update(drawn, alive, visuals, viewer.camera()->projectionMatrix->transform(), viewer.lookAt()->eye, *ellipsoid, sunDir);
                detonations.update(drawnTime, viewer.camera()->projectionMatrix->transform(), viewer.lookAt()->eye, *ellipsoid, sunDir);
                if (flow) {
                    flow->update(drawnTime, drawn, alive, visuals, viewer.camera()->projectionMatrix->transform(), viewer.lookAt()->eye,
                                 *ellipsoid, sunDir);
                    if (opt.stats > 0.0) {
                        const auto& c = flow->cost();
                        stats.flowGpu += c.solverGpuMs, stats.flowCpu += c.cpuMs, stats.flowPack += c.packMs;
                        stats.flowGpuMax = std::max(stats.flowGpuMax, c.solverGpuMs), stats.flowCpuMax = std::max(stats.flowCpuMax, c.cpuMs);
                        stats.flowLeaves = std::max<std::uint64_t>(stats.flowLeaves, c.leaves);
                        stats.flowPages = std::max<std::uint64_t>(stats.flowPages, c.pages);
                        ++stats.flowFrames;
                    }
                }
                vapour.update(drawn, alive, visuals, viewer.camera()->viewMatrix->transform(), viewer.camera()->projectionMatrix->transform(),
                              viewer.lookAt()->eye, sunDir, light);
            }
        }
        // the vehicles' depth, drawn while the vapour reads it
        if (auto pass = viewer.depthPrepass().second) pass->setAllChildren(vapour.wantsSolid());
        {
            const vsg::dvec3 lla = ellipsoid->convertECEFToLatLongAltitude(viewer.lookAt()->eye);
            controls->eyeLatDeg.store(lla.x, std::memory_order_relaxed);
            controls->eyeLonDeg.store(lla.y, std::memory_order_relaxed);
            controls->eyeAltM.store(lla.z, std::memory_order_relaxed);
            controls->eyeDistanceM.store(camera->distance(), std::memory_order_relaxed);
        }

        if (opt.trace >= 0 && batch && static_cast<std::size_t>(opt.trace) < batch->states.size()) {
            static double lastTrace = -1.0;
            if (batch->simTime - lastTrace >= 1.0) {
                lastTrace = batch->simTime;
                const auto& st = batch->states[static_cast<std::size_t>(opt.trace)];
                std::printf("trace t=%6.2f v%d lat %.5f lon %.5f alt %8.2f agl %8.2f tas %7.2f vz %7.2f roll %6.1f pitch %6.1f ground %d%s\n",
                            batch->simTime, opt.trace, units::radiansToDegrees(st.latitudeRad), units::radiansToDegrees(st.longitudeRad),
                            st.altitudeMslM, st.altitudeAglM, st.airspeedTrueMs, -st.velocityNedMs[2],
                            units::radiansToDegrees(st.eulerRad[0]), units::radiansToDegrees(st.eulerRad[1]), st.onGround ? 1 : 0,
                            st.diverged ? " DIVERGED" : "");
                std::fflush(stdout);
            }
        }
        static int tileTick = 0;
        if (terrain && batch && ++tileTick % 60 == 0)
            for (const auto& st : batch->states) terrain->requestAround(st.latitudeRad, st.longitudeRad);
        gui->setBatch(batch);
        gui->setStatus(mirror.valid() ? &mirror.status() : nullptr);
        cameraPreview->update();
        {
            const std::size_t at = static_cast<std::size_t>(std::max(selected, 0));
            missionGui->update(mirror.valid() ? &mirror : nullptr, anyAlive ? selected : -1, at < meta.size() ? meta[at].name : std::string(),
                               batch ? batch->simTime : 0.0);
        }
        controls->fps.store(viewer.fps(), std::memory_order_relaxed);
        controls->frameMs.store(viewer.frameSeconds() * 1e3, std::memory_order_relaxed);

        if (!viewer.frame()) break;
        if (opt.stats > 0.0 && viewer.window())
            gpuTimer.frameDone(*viewer.window()->getOrCreateDevice(),
                               viewer.window()->getOrCreatePhysicalDevice()->getProperties().limits.timestampPeriod);
        if (!opt.screenshot.empty() && opt.screenshotZoom != 1.0 && wallSeconds >= opt.screenshotAfter - 1.0)
            camera->zoom(std::pow(opt.screenshotZoom, viewer.frameSeconds()));
        if (!opt.screenshot.empty() && opt.screenshotOrbit != 0.0 && wallSeconds >= opt.screenshotAfter - 1.0)
            camera->turn(opt.screenshotOrbit * viewer.frameSeconds() * 3.141592653589793 / 180.0);
        if (!opt.screenshot.empty() &&
            (shotsOnReplay ? replayTime >= opt.screenshotAfter + shots * opt.screenshotEvery - 1e-9
                           : wallSeconds >= opt.screenshotAfter + shots * opt.screenshotEvery)) {
            if (opt.screenshotFrames == 1) {
                viewer.screenshot(opt.screenshot);
                break;
            }
            // a sequence: <stem>_000<ext>, <stem>_001<ext>, ...
            const std::filesystem::path path(opt.screenshot);
            char number[16];
            std::snprintf(number, sizeof number, "_%03d", shots);
            viewer.screenshot((path.parent_path() / (path.stem().string() + number + path.extension().string())).string());
            if (++shots >= opt.screenshotFrames) break;
        }

        if (opt.stats > 0.0) {
            const auto& t = viewer.timing();
            stats.window += viewer.frameSeconds();
            if (viewer.frameSeconds() > stats.maxFrame) stats.maxFrame = viewer.frameSeconds(), stats.worst = t;
            // late: over 1.5 refresh intervals (the last window's mean frame, when it was vsync-bound)
            if (stats.lastMean > 0.0 && viewer.frameSeconds() > 1.5 * stats.lastMean) ++stats.late;
            ++stats.frames;
            stats.sum.app += t.app; stats.sum.advance += t.advance; stats.sum.events += t.events; stats.sum.update += t.update;
            stats.sum.record += t.record; stats.sum.present += t.present; stats.sum.sleep += t.sleep;
            if (stats.window >= 1.0) {
                const double cpu = platform::processCpuSeconds();
                const double n = static_cast<double>(stats.frames);
                std::printf("stats t=%5.1f  %5.1f fps  frame avg %5.2f max %5.2f ms | app %4.2f advance %5.2f events %4.2f update %4.2f record %4.2f present %5.2f sleep %5.2f ms | cpu %5.1f%% | vehicles %zu\n",
                            wallSeconds, n / stats.window, stats.window / n * 1e3, stats.maxFrame * 1e3, stats.sum.app / n * 1e3, stats.sum.advance / n * 1e3,
                            stats.sum.events / n * 1e3, stats.sum.update / n * 1e3, stats.sum.record / n * 1e3, stats.sum.present / n * 1e3,
                            stats.sum.sleep / n * 1e3, (cpu - stats.cpu0) / stats.window * 100.0,
                            static_cast<std::size_t>(std::count(alive.begin(), alive.end(), 1)));
                const auto& w = stats.worst;
                std::printf("worst | app %.2f advance %.2f events %.2f update %.2f record %.2f present %.2f sleep %.2f ms | %d late\n",
                            w.app * 1e3, w.advance * 1e3, w.events * 1e3, w.update * 1e3, w.record * 1e3, w.present * 1e3, w.sleep * 1e3,
                            stats.late);
                if (!gpuTimer.empty()) std::printf("%s\n", gpuTimer.report().c_str());
                if (stats.flowFrames > 0) {
                    const double m = static_cast<double>(stats.flowFrames);
                    std::printf("flow | solver gpu avg %.2f max %.2f ms | cpu avg %.2f max %.2f ms (packing %.2f) | leaves %llu, pages %llu\n",
                                stats.flowGpu / m, stats.flowGpuMax, stats.flowCpu / m, stats.flowCpuMax, stats.flowPack / m,
                                static_cast<unsigned long long>(stats.flowLeaves), static_cast<unsigned long long>(stats.flowPages));
                }
                std::fflush(stdout);
                const double mean = stats.window / n;
                stats = Stats{};
                stats.cpu0 = cpu;
                stats.lastMean = mean;
            }
            if (wallSeconds >= opt.stats) break;
        }

        if (opt.probe && batch && runner) {
            static std::vector<double> speeds, distances;
            static vsg::dvec3 lastPos;
            static bool havePos = false;
            static int warmup = 60;
            if (warmup > 0) { --warmup; havePos = false; }
            const auto& st = (opt.interpolate ? interpolator.states() : batch->states)[static_cast<std::size_t>(selected)];
            const vsg::dvec3 pos(st.positionEcef[0], st.positionEcef[1], st.positionEcef[2]);
            if (havePos && viewer.frameSeconds() > 0.0) {
                // Per displayed frame (the monitor's cadence), not per CPU-measured frame time.
                speeds.push_back(vsg::length(pos - lastPos) * viewer.fps());
                distances.push_back(vsg::length(viewer.lookAt()->eye - pos));
                if (std::getenv("FSIM_PROBE_TRACE") && speeds.size() <= 60)
                    std::printf("frame %3zu dt %6.3f ms  speed %6.2f  renderTime %.4f  simTime %.4f\n", speeds.size(), viewer.frameSeconds() * 1e3,
                                speeds.back(), interpolator.renderSimTime(), batch->simTime);
            }
            lastPos = pos;
            havePos = true;
            if (speeds.size() >= 600) {
                auto summarise = [](const std::vector<double>& v) {
                    double mean = 0, m2 = 0, lo = 1e300, hi = -1e300;
                    for (double x : v) { mean += x; lo = std::min(lo, x); hi = std::max(hi, x); }
                    mean /= static_cast<double>(v.size());
                    for (double x : v) m2 += (x - mean) * (x - mean);
                    return std::tuple{mean, std::sqrt(m2 / static_cast<double>(v.size())), lo, hi};
                };
                const auto [sm, ss, slo, shi] = summarise(speeds);
                const auto [dm, ds, dlo, dhi] = summarise(distances);
                std::printf("probe (%s, %zu frames): apparent speed mean %.2f m/s sd %.2f min %.2f max %.2f | "
                            "eye-target mean %.2f m sd %.4f min %.2f max %.2f | sim %.0f veh-steps/s simTime %.2f tas %.1f fps %.0f\n",
                            opt.interpolate ? "interpolated" : "sample-and-hold", speeds.size(), sm, ss, slo, shi, dm, ds, dlo, dhi,
                            runner->throughput(), batch->simTime, st.airspeedTrueMs, viewer.fps());
                break;
            }
        }
    }

    if (runner) runner->stop();

    // Leave without unwinding. Freeing a deep tile pyramid takes seconds -
    // thousands of tiles, each with image and elevation arrays, GPU buffers
    // and descriptor sets, released one at a time - and the window has
    // stopped drawing by then, so it reads as a hang: closing the viewer used
    // to sit there for about six seconds before the process went away.
    //
    // Nothing here owns state that has to outlive the process. The viewer
    // only ever reads the world registry, it publishes nothing, the sim
    // runner has already been stopped and joined, and quiesce() has waited
    // for the tile cache writes. The operating system reclaims the memory,
    // the GPU resources and the handles.
    viewer.quiesce();
    std::fflush(nullptr); // the log and any --probe/--stats output go to stdio
    std::_Exit(0);
}
