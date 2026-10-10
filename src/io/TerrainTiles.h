#pragma once

#include "fsim/GroundProvider.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fsim::io {

/// A decoded elevation tile: `size` x `size` metres above the ellipsoid, row 0 = north.
struct ElevationTile {
    std::uint32_t size = 0;
    std::vector<float> heights;
    bool valid() const noexcept { return size >= 2 && heights.size() == static_cast<std::size_t>(size) * size; }
};

/// Decode a Terrarium PNG (AWS terrain tiles: h = R*256 + G + B/256 - 32768)
/// into metres. Bathymetry is clamped to sea level, matching the viewer.
/// Returns an invalid tile if the bytes are not a decodable image.
ElevationTile decodeTerrariumPng(const std::vector<std::uint8_t>& png);

/// Read a PNG file as one grey channel (row 0 its top). False when it cannot be read.
bool readGreyPng(const std::filesystem::path& path, std::vector<std::uint8_t>& pixels, std::uint32_t& width, std::uint32_t& height);
/// The same as four channels, red green blue alpha (row 0 its top).
bool readRgbaPng(const std::filesystem::path& path, std::vector<std::uint8_t>& pixels, std::uint32_t& width, std::uint32_t& height);

/// Web-Mercator tile coordinates of a geodetic position at `zoom` (continuous:
/// integer part = tile index, fraction = position within the tile).
void mercatorTile(double latitudeRad, double longitudeRad, unsigned zoom, double& tx, double& ty) noexcept;

/// Physics ground from public elevation tiles, headless (design 7.3, ADR-7):
/// the same AWS Terrarium tiles the viewer drapes its imagery over, fetched
/// with the platform HTTP client, kept in a disk cache the viewer shares
/// (`<maps>/<host>/<path>`: platform::mapsDir()), decoded into an LRU. Sampling is
/// bilinear. A first visit to a tile costs a download (~0.2-1.5 s); use
/// `prefetch()` around spawn points and `requestAround()` as vehicles move.
class TerrainTiles final : public sim::GroundProvider {
public:
    struct Options {
        std::string urlTemplate = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png";
        /// 15: the deepest level of the pyramid, ~3.8 m/px at mid latitudes - the raster the viewer draws close up
        /// (world::EarthSettings::elevationMaxLevel; deeper levels are upsampled from it). Physics stands on what is
        /// drawn: at level 12 the ground stood up to 20 m off the drawn hillsides (docs/flight-autonomy.md, 4.68).
        unsigned zoom = 15;
        std::filesystem::path cacheDir;         ///< empty = the local map set (platform::mapsDir())
        std::size_t cacheTiles = 512;           ///< decoded tiles kept in memory (~260 KB each)
        /// Resample each tile to this many samples per edge, as the renderer
        /// resamples its mesh (world::EarthSettings::elevationMeshDimension).
        /// 0 keeps the raster at full resolution, which is what the physics
        /// wants: it asks where the ground *is*, not where it is drawn. The
        /// camera wants the drawn surface, because flying into scenery it can
        /// see is the thing anyone notices.
        std::uint32_t meshDimension = 0;
        /// Read only what is already in cacheDir; never fetch - the default:
        /// everything there is to stand on was put in the map set beforehand
        /// (fetch-maps / tile_prefetch), and a missing tile is its nearest
        /// ancestor there. False lets it download what is missing.
        bool offline = true;
        unsigned loaderThreads = 2;
    };
    /// Test hook: replaces the HTTP fetch (bytes of the tile at z/x/y).
    using Fetch = std::function<bool(unsigned z, unsigned x, unsigned y, std::vector<std::uint8_t>& png)>;

    explicit TerrainTiles(Options options, Fetch fetch = {});
    ~TerrainTiles() override;

    double heightAboveEllipsoidM(double latitudeRad, double longitudeRad) const override;
    /// Empty where its tile cannot be loaded (offline, and not on disk).
    std::optional<double> knownHeightAboveEllipsoidM(double latitudeRad, double longitudeRad) const override;
    /// A tile's pixel at the equator (a Terrarium tile is 256 across): 38 m at zoom 12.
    double resolutionM() const noexcept override;
    /// A profile read from the data itself: offline, each point from the deepest level the map set holds there (what
    /// this level's tile there is made from), a source tile at a time, decoded once - never the tiles of this level a
    /// missing level makes (a 65,536-texel resample each, hundreds along a long path) nor the cache the vehicles stand
    /// on. Online, this level's tiles, a tile at a time.
    void profile(double lat0, double lon0, double lat1, double lon1, int n, double spacingM, double* out) const override;
    /// heightAboveEllipsoidM's value, to the bit, without making or caching this level's tile: from the vehicles' cache
    /// where it holds the tile; else, offline, the map set's own tile here or - where this level is missing - the four
    /// texels of the tile fromAncestor() would make that the point reads, from the source cache (decoded once).
    double pointHeightAboveEllipsoidM(double latitudeRad, double longitudeRad) const override;

    /// Height if the tile is already decoded (never blocks); else nullopt and
    /// the tile is queued for background loading.
    std::optional<double> cachedHeightAboveEllipsoidM(double latitudeRad, double longitudeRad);

    /// Load every tile within `radiusM` of a position (blocking, parallel).
    void prefetch(double latitudeRad, double longitudeRad, double radiusM, unsigned threads = 8);
    /// Queue the tile under a position and its 8 neighbours for background loading.
    void requestAround(double latitudeRad, double longitudeRad);

    unsigned zoom() const noexcept { return options_.zoom; }
    std::size_t cachedTiles() const;
    std::size_t misses() const noexcept { return misses_.load(); }
    std::size_t failures() const noexcept { return failures_.load(); }

private:
    struct Key {
        unsigned x, y;
        bool operator==(const Key& o) const noexcept { return x == o.x && y == o.y; }
    };
    struct KeyHash {
        std::size_t operator()(const Key& k) const noexcept { return (static_cast<std::size_t>(k.x) << 32) ^ k.y; }
    };
    using Tile = std::shared_ptr<const ElevationTile>;
    /// Decoded tiles of the map set, by level and position, shared by every level read from it (an offline set's
    /// missing levels are each made from an ancestor: one decode of the ancestor, not one a descendant), and the files
    /// it does not have.
    struct Sources {
        std::mutex mutex;
        std::unordered_map<std::uint64_t, std::pair<Tile, std::list<std::uint64_t>::iterator>> decoded;
        std::list<std::uint64_t> lru;
        std::unordered_set<std::uint64_t> missing;
        std::size_t capacity = 192; ///< (~260 KB each: the source tiles of a region's radio paths)
    };
    Tile source(unsigned z, unsigned x, unsigned y) const;

    Tile tile(unsigned x, unsigned y) const;
    Tile load(unsigned x, unsigned y) const;
    /// Offline, with this level missing: the nearest ancestor on disk,
    /// restricted to this tile (see load()).
    Tile fromAncestor(unsigned x, unsigned y) const;
    std::string url(unsigned x, unsigned y) const;
    std::string urlAt(unsigned z, unsigned x, unsigned y) const;
    std::filesystem::path cachePath(unsigned x, unsigned y) const;
    std::filesystem::path cachePathAt(unsigned z, unsigned x, unsigned y) const;
    /// Bilinear at (tx, ty)'s fraction of the tile. `centres`: a raster - pixel j's centre at (j + 0.5) / size, the
    /// outer half pixel extrapolated from the edge, as the viewer's upsampler reads it; else a vertex grid (sample k
    /// at k / (size - 1)), as the mesh is built.
    static double sample(const ElevationTile& t, double tx, double ty, bool centres) noexcept;
    /// Texel (i, j) of the tile fromAncestor() makes for physics from `raw`, its ancestor `d` levels up (the tile's
    /// place within it qx, qy; share 2^-d): this level's tile's share of the ancestor's raster at its texel centres.
    static float ancestorTexel(const ElevationTile& raw, unsigned qx, unsigned qy, double share, std::uint32_t i, std::uint32_t j) noexcept;
    void ensureLoaders();
    void loaderLoop();

    Options options_;
    Fetch fetch_;
    mutable std::mutex mutex_;
    mutable std::unordered_map<Key, std::pair<Tile, std::list<Key>::iterator>, KeyHash> cache_;
    mutable std::list<Key> lru_;
    mutable std::atomic<std::size_t> misses_{0}, failures_{0};

    std::mutex queueMutex_;
    std::condition_variable queueCv_;
    std::deque<Key> queue_;
    std::unordered_set<std::uint64_t> queued_;
    std::vector<std::thread> loaders_;
    std::atomic<bool> stopLoaders_{false};

    std::shared_ptr<Sources> sources_ = std::make_shared<Sources>();
};

} // namespace fsim::io
