#pragma once

// The native World's construction options: the values the runtime is built from (session::World). A neutral contract
// (src/contracts, ARCH-02): they sit below the runtime and below the mapping from the public fsim::WorldOptions
// (session/WorldOptions.h, which ARCH-04 moves to the lower simulation facade), so neither the mapper nor its public
// inputs enter the runtime target. The name, namespace, fields, order and defaults are those it had in session/World.h.

#include "fsim/GroundProvider.h"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace fsim::session {

struct WorldOptions {
    std::string name = "default";       ///< shared-memory world name (viewers attach by it)
    double dt = 1.0 / 120.0;            ///< FDM step
    int frameSkip = 4;                  ///< FDM steps per world step
    unsigned workers = 0;               ///< 0 = physical cores - 2
    bool pinWorkers = true;             ///< one worker per physical core (throughput stability)
    std::uint64_t seed = 0;
    std::uint32_t capacity = 256;       ///< published vehicle slots
    bool publish = true;                ///< false = invisible to viewers
    bool storeDrag = true;              ///< carried stores' and their carriage's drag on the aircraft (MS-7g); false: their mass only
    double publishIntervalSeconds = 1.0 / 60.0;
    std::filesystem::path jsbsimRoot;   ///< empty = auto-detect
    std::shared_ptr<sim::GroundProvider> ground; ///< custom ground; null = `terrain` below or flat at 0 m
    bool terrain = false;               ///< physics ground from the public elevation tiles the viewer draws
    std::string terrainUrl;             ///< tile template; empty = AWS Terrarium
    unsigned terrainZoom = 15;          ///< ~3.8 m/px: the raster the viewer draws close up (io::TerrainTiles::Options::zoom)
    double terrainPrefetchRadiusM = 1500.0; ///< tiles loaded (blocking) around each new vehicle's position (then ahead of it, in the background)
    bool terrainOffline = false;        ///< the tiles from the cache alone, never fetched (io::TerrainTiles::Options::offline)
    std::filesystem::path recordPath;   ///< non-empty: write every vehicle's state to this file for replay in the viewer
    double recordIntervalSeconds = 0.0; ///< simulation time between recorded frames; 0 = every world step
};

} // namespace fsim::session
