#include "foundation/core/HeightGrid.h"
#include "foundation/io/TerrainTiles.h"

#include "foundation/core/Log.h"
#include "foundation/platform/Http.h"
#include "foundation/platform/Paths.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#include <stb_image.h>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace fsim::io {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

namespace {
bool readPng(const std::filesystem::path& path, int channels, std::vector<std::uint8_t>& pixels, std::uint32_t& width, std::uint32_t& height) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    const std::vector<std::uint8_t> png((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    int w = 0, h = 0, has = 0;
    unsigned char* data = stbi_load_from_memory(png.data(), static_cast<int>(png.size()), &w, &h, &has, channels);
    if (!data || w < 1 || h < 1) {
        if (data) stbi_image_free(data);
        return false;
    }
    width = static_cast<std::uint32_t>(w);
    height = static_cast<std::uint32_t>(h);
    pixels.assign(data, data + static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * static_cast<std::size_t>(channels));
    stbi_image_free(data);
    return true;
}
} // namespace

bool readGreyPng(const std::filesystem::path& path, std::vector<std::uint8_t>& pixels, std::uint32_t& width, std::uint32_t& height) {
    return readPng(path, 1, pixels, width, height);
}

bool readRgbaPng(const std::filesystem::path& path, std::vector<std::uint8_t>& pixels, std::uint32_t& width, std::uint32_t& height) {
    return readPng(path, 4, pixels, width, height);
}

void mercatorTile(double latRad, double lonRad, unsigned zoom, double& tx, double& ty) noexcept {
    const double n = static_cast<double>(1u << zoom);
    const double lat = std::clamp(latRad, -1.4844222297453322, 1.4844222297453322); // +-85.0511 deg
    tx = (lonRad / kPi + 1.0) * 0.5 * n;
    ty = (1.0 - std::log(std::tan(lat) + 1.0 / std::cos(lat)) / kPi) * 0.5 * n;
    if (tx < 0.0) tx += n;
    if (tx >= n) tx -= n;
}

ElevationTile decodeTerrariumPng(const std::vector<std::uint8_t>& png) {
    ElevationTile tile;
    int w = 0, h = 0, channels = 0;
    unsigned char* pixels = stbi_load_from_memory(png.data(), static_cast<int>(png.size()), &w, &h, &channels, 3);
    if (!pixels || w < 2 || w != h) {
        if (pixels) stbi_image_free(pixels);
        return tile;
    }
    tile.size = static_cast<std::uint32_t>(w);
    tile.heights.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
    for (std::size_t i = 0; i < tile.heights.size(); ++i) {
        const unsigned char* p = pixels + i * 3;
        // Sea floor clamped to 0: no water surface is drawn, and the viewer does the same.
        tile.heights[i] = std::max(0.0f, static_cast<float>(p[0]) * 256.0f + static_cast<float>(p[1]) + static_cast<float>(p[2]) / 256.0f - 32768.0f);
    }
    stbi_image_free(pixels);
    return tile;
}

TerrainTiles::TerrainTiles(Options options, Fetch fetch) : options_(std::move(options)), fetch_(std::move(fetch)) {
    options_.zoom = std::min(options_.zoom, 22u);
    options_.cacheTiles = std::max<std::size_t>(options_.cacheTiles, 8);
    if (options_.cacheDir.empty()) options_.cacheDir = platform::mapsDir();
}

TerrainTiles::~TerrainTiles() {
    stopLoaders_.store(true);
    queueCv_.notify_all();
    for (auto& t : loaders_) t.join();
}

std::string TerrainTiles::url(unsigned x, unsigned y) const { return urlAt(options_.zoom, x, y); }

std::string TerrainTiles::urlAt(unsigned z, unsigned x, unsigned y) const {
    std::string s = options_.urlTemplate;
    auto sub = [&](const char* key, unsigned v) {
        for (auto pos = s.find(key); pos != std::string::npos; pos = s.find(key)) s.replace(pos, 3, std::to_string(v));
    };
    sub("{z}", z);
    sub("{x}", x);
    sub("{y}", y);
    return s;
}

std::filesystem::path TerrainTiles::cachePath(unsigned x, unsigned y) const { return cachePathAt(options_.zoom, x, y); }

std::filesystem::path TerrainTiles::cachePathAt(unsigned z, unsigned x, unsigned y) const {
    // Same layout as vsgXchange's file cache (<cache>/<host>/<path>) so the
    // viewer and a training application share downloads.
    std::string u = urlAt(z, x, y);
    const auto scheme = u.find("://");
    if (scheme != std::string::npos) u = u.substr(scheme + 3);
    return options_.cacheDir / std::filesystem::path(u);
}

namespace {

// A tile fromAncestor() makes for physics, texel by texel (and pointHeightAboveEllipsoidM's texels, the same arithmetic):
// texel i's centre in the ancestor's raster (the tile at q within it, its share of it 2^-d)...
double ancestorCoord(unsigned q, std::uint32_t i, std::uint32_t n, double share) noexcept {
    return (static_cast<double>(q) + (static_cast<double>(i) + 0.5) / n) * share * n - 0.5;
}
// ...the raster's texel before it and the fraction past it, held to its edge texels...
void ancestorAxis(double s, std::uint32_t n, std::uint32_t& a, double& t) noexcept {
    const double hi = static_cast<double>(n - 1);
    a = static_cast<std::uint32_t>(std::min(std::floor(std::clamp(s, 0.0, hi)), hi - 1.0));
    t = std::clamp(s - static_cast<double>(a), 0.0, 1.0);
}
// ...and the bilinear between the four
float ancestorBlend(const ElevationTile& raw, std::uint32_t x0, double tx, std::uint32_t y0, double ty) noexcept {
    const std::uint32_t n = raw.size;
    auto at = [&](std::uint32_t a, std::uint32_t b) { return static_cast<double>(raw.heights[static_cast<std::size_t>(b) * n + a]); };
    const double h = (at(x0, y0) * (1 - tx) + at(x0 + 1, y0) * tx) * (1 - ty) + (at(x0, y0 + 1) * (1 - tx) + at(x0 + 1, y0 + 1) * tx) * ty;
    return static_cast<float>(h);
}

} // namespace

TerrainTiles::Tile TerrainTiles::source(unsigned z, unsigned x, unsigned y) const {
    // (an ancestor serves up to 4^d tiles of this level: read and decoded once, not once each)
    const std::uint64_t key = (static_cast<std::uint64_t>(z) << 48) | (static_cast<std::uint64_t>(x) << 24) | y;
    Sources& s = *sources_;
    {
        std::lock_guard<std::mutex> lock(s.mutex);
        if (s.missing.count(key)) return nullptr;
        if (auto it = s.decoded.find(key); it != s.decoded.end()) {
            s.lru.splice(s.lru.begin(), s.lru, it->second.second);
            return it->second.first;
        }
    }
    std::ifstream in(cachePathAt(z, x, y), std::ios::binary);
    Tile raw;
    if (in) {
        const std::vector<std::uint8_t> png{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
        raw = std::make_shared<ElevationTile>(decodeTerrariumPng(png));
    }
    std::lock_guard<std::mutex> lock(s.mutex);
    if (!raw) {
        if (s.missing.size() > 200000) s.missing.clear(); // (bounded: a long tour forgets what it looked for first)
        s.missing.insert(key);
        return nullptr;
    }
    if (auto it = s.decoded.find(key); it != s.decoded.end()) return it->second.first; // (another worker's)
    s.lru.push_front(key);
    s.decoded.emplace(key, std::make_pair(raw, s.lru.begin()));
    while (s.decoded.size() > s.capacity) {
        s.decoded.erase(s.lru.back());
        s.lru.pop_back();
    }
    return raw;
}

TerrainTiles::Tile TerrainTiles::fromAncestor(unsigned x, unsigned y) const {
    for (unsigned d = 1; d <= options_.zoom; ++d) {
        const unsigned az = options_.zoom - d, ax = x >> d, ay = y >> d;
        const Tile source = this->source(az, ax, ay);
        if (!source) continue;
        const ElevationTile& raw = *source;
        if (!raw.valid()) return nullptr;
        const unsigned mask = (1u << d) - 1u, qx = x & mask, qy = y & mask;
        const double share = 1.0 / static_cast<double>(1u << d);
        auto tile = std::make_shared<ElevationTile>();

        if (options_.meshDimension > 0 && raw.size > options_.meshDimension) {
            // The camera: the ancestor's *drawn* surface, restricted to this
            // tile. Where there is nothing deeper, the renderer stops refining
            // at the ancestor's level and draws exactly its vertex grid, so
            // interpolating between those vertices is what is on screen - not
            // the raster resampled afresh at this tile's size, which filters
            // over a smaller window and puts valley floors metres lower than
            // they are drawn.
            const std::uint32_t g = options_.meshDimension;
            const auto grid = core::resampleHeightGrid(raw.heights.data(), raw.size, g);
            tile->size = g;
            tile->heights.resize(static_cast<std::size_t>(g) * g);
            const double last = static_cast<double>(g - 1);
            for (std::uint32_t j = 0; j < g; ++j) {
                const double fy = std::clamp((static_cast<double>(qy) + static_cast<double>(j) / last) * share * last, 0.0, last);
                const auto y0 = static_cast<std::uint32_t>(std::min(std::floor(fy), last - 1.0));
                const double ty = fy - static_cast<double>(y0);
                for (std::uint32_t i = 0; i < g; ++i) {
                    const double fx = std::clamp((static_cast<double>(qx) + static_cast<double>(i) / last) * share * last, 0.0, last);
                    const auto x0 = static_cast<std::uint32_t>(std::min(std::floor(fx), last - 1.0));
                    const double tx = fx - static_cast<double>(x0);
                    auto at = [&](std::uint32_t a, std::uint32_t b) {
                        return static_cast<double>(grid[static_cast<std::size_t>(b) * g + a]);
                    };
                    const double h = (at(x0, y0) * (1 - tx) + at(x0 + 1, y0) * tx) * (1 - ty) +
                                     (at(x0, y0 + 1) * (1 - tx) + at(x0 + 1, y0 + 1) * tx) * ty;
                    tile->heights[static_cast<std::size_t>(j) * g + i] = static_cast<float>(h);
                }
            }
        } else {
            // Physics: the ground where it is, at the ancestor's resolution -
            // this tile's share of its raster, texel centres as a real tile has
            // (ancestorTexel, a row at a time).
            const std::uint32_t n = raw.size;
            tile->size = n;
            tile->heights.resize(static_cast<std::size_t>(n) * n);
            for (std::uint32_t j = 0; j < n; ++j) {
                std::uint32_t y0;
                double ty;
                ancestorAxis(ancestorCoord(qy, j, n, share), n, y0, ty);
                for (std::uint32_t i = 0; i < n; ++i) {
                    std::uint32_t x0;
                    double tx;
                    ancestorAxis(ancestorCoord(qx, i, n, share), n, x0, tx);
                    tile->heights[static_cast<std::size_t>(j) * n + i] = ancestorBlend(raw, x0, tx, y0, ty);
                }
            }
        }
        return tile;
    }
    return nullptr;
}

TerrainTiles::Tile TerrainTiles::load(unsigned x, unsigned y) const {
    std::vector<std::uint8_t> png;
    const auto path = cachePath(x, y);
    bool fromDisk = false;
    if (!fetch_) {
        std::ifstream in(path, std::ios::binary);
        if (in) {
            png.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
            fromDisk = !png.empty();
        }
    }
    if (!fromDisk && options_.offline) {
        // An offline set is partial: this level may be missing where a coarser
        // one is not. Answering "no ground" there left the camera with no
        // clearance and the focus at 0 m inside the Alps, whose elevation stops
        // at level 11 while the camera asks for 12.
        if (auto ancestor = fromAncestor(x, y)) return ancestor;
        failures_.fetch_add(1);
        return nullptr;
    }
    if (!fromDisk) {
        bool ok = false;
        if (fetch_) {
            ok = fetch_(options_.zoom, x, y, png);
        } else {
            std::string error;
            for (int attempt = 0; attempt < 3 && !ok; ++attempt) ok = platform::httpGet(url(x, y), png, &error);
            if (!ok) LOG_WARN("io") << "elevation tile " << url(x, y) << ": " << error;
        }
        if (!ok) {
            failures_.fetch_add(1);
            return nullptr;
        }
    }
    auto decoded = std::make_shared<ElevationTile>(decodeTerrariumPng(png));
    if (!decoded->valid()) {
        LOG_WARN("io") << "elevation tile " << url(x, y) << ": not a decodable PNG";
        failures_.fetch_add(1);
        return nullptr;
    }
    // Serve the surface the renderer draws, when asked. The mesh is built from
    // a box-filtered, resampled copy of this raster, and a box filter lowers
    // peaks and raises valley floors: over 40 tiles of real relief the drawn
    // mesh sat up to 11.3 m above the raw raster, more than the clearance the
    // camera keeps at close range, so the eye finished up inside the hillside.
    if (options_.meshDimension > 0 && decoded->size > options_.meshDimension) {
        auto resampled = core::resampleHeightGrid(decoded->heights.data(), decoded->size, options_.meshDimension);
        if (!resampled.empty()) {
            decoded->heights = std::move(resampled);
            decoded->size = options_.meshDimension;
        }
    }
    if (!fromDisk && !fetch_) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
        std::ofstream out(path, std::ios::binary);
        if (out) out.write(reinterpret_cast<const char*>(png.data()), static_cast<std::streamsize>(png.size()));
    }
    return decoded;
}

TerrainTiles::Tile TerrainTiles::tile(unsigned x, unsigned y) const {
    const Key key{x, y};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (auto it = cache_.find(key); it != cache_.end()) {
            lru_.splice(lru_.begin(), lru_, it->second.second);
            return it->second.first;
        }
    }
    Tile t = load(x, y); // outside the lock: other workers keep sampling cached tiles
    std::lock_guard<std::mutex> lock(mutex_);
    misses_.fetch_add(1);
    if (auto it = cache_.find(key); it != cache_.end()) return it->second.first; // another worker won the race
    lru_.push_front(key);
    cache_.emplace(key, std::make_pair(t, lru_.begin()));
    while (cache_.size() > options_.cacheTiles) {
        cache_.erase(lru_.back());
        lru_.pop_back();
    }
    return t;
}

namespace {

/// sample()'s bilinear reading of a `size`-texel tile at (tx, ty)'s fraction, its texels from `at(x, y)`.
template <class At>
double bilinear(std::uint32_t size, double tx, double ty, bool centres, At&& at) {
    // A raster's pixel is an area, its height at its centre: read as a vertex grid (pixel j at j / (size - 1)) the
    // ground shifted by up to half a pixel towards the tile's edges - 2.4 m at level 15, 19 m at level 12 - away from
    // the surface the viewer draws (its upsampler reads pixel centres, world/ElevationUpsampler.cpp)
    const double n = static_cast<double>(size), hi = n - 1.0;
    const double fx = centres ? (tx - std::floor(tx)) * n - 0.5 : (tx - std::floor(tx)) * hi;
    const double fy = centres ? (ty - std::floor(ty)) * n - 0.5 : (ty - std::floor(ty)) * hi;
    const auto x0 = static_cast<std::uint32_t>(std::min(std::floor(std::max(fx, 0.0)), hi - 1.0));
    const auto y0 = static_cast<std::uint32_t>(std::min(std::floor(std::max(fy, 0.0)), hi - 1.0));
    const std::uint32_t x1 = x0 + 1, y1 = y0 + 1;
    const double ax = fx - x0, ay = fy - y0; // (outside [0, 1] in the outer half pixel: extrapolated)
    return (at(x0, y0) * (1 - ax) + at(x1, y0) * ax) * (1 - ay) + (at(x0, y1) * (1 - ax) + at(x1, y1) * ax) * ay;
}

} // namespace

double TerrainTiles::sample(const ElevationTile& t, double tx, double ty, bool centres) noexcept {
    return bilinear(t.size, tx, ty, centres,
                    [&](std::uint32_t x, std::uint32_t y) { return static_cast<double>(t.heights[static_cast<std::size_t>(y) * t.size + x]); });
}

float TerrainTiles::ancestorTexel(const ElevationTile& raw, unsigned qx, unsigned qy, double share, std::uint32_t i, std::uint32_t j) noexcept {
    std::uint32_t x0, y0;
    double tx, ty;
    ancestorAxis(ancestorCoord(qy, j, raw.size, share), raw.size, y0, ty);
    ancestorAxis(ancestorCoord(qx, i, raw.size, share), raw.size, x0, tx);
    return ancestorBlend(raw, x0, tx, y0, ty);
}

double TerrainTiles::pointHeightAboveEllipsoidM(double latitudeRad, double longitudeRad) const {
    const bool centres = options_.meshDimension == 0;
    if (!options_.offline || fetch_ || !centres) return heightAboveEllipsoidM(latitudeRad, longitudeRad); // (a tile fetched, or the camera's)
    double tx, ty;
    mercatorTile(latitudeRad, longitudeRad, options_.zoom, tx, ty);
    const double tiles = static_cast<double>(1u << options_.zoom);
    const auto x = static_cast<unsigned>(std::floor(tx)), y = static_cast<unsigned>(std::clamp(std::floor(ty), 0.0, tiles - 1.0));
    { // (the vehicles' cache holds this tile: read it, its place in their LRU left as it is)
        std::lock_guard<std::mutex> lock(mutex_);
        if (auto it = cache_.find(Key{x, y}); it != cache_.end()) {
            const Tile& t = it->second.first;
            return t && t->valid() ? sample(*t, tx, ty, centres) : 0.0;
        }
    }
    // this level's tile on disk: decoded as load() decodes it (an empty or broken file: load()'s own answer)
    if (const Tile own = source(options_.zoom, x, y)) return own->valid() ? sample(*own, tx, ty, centres) : heightAboveEllipsoidM(latitudeRad, longitudeRad);
    // missing: the tile fromAncestor() would make here - the nearest ancestor on disk - read at the four texels the point
    // takes (none: no ground, sea level, as heightAboveEllipsoidM reads a tile it cannot load)
    for (unsigned d = 1; d <= options_.zoom; ++d) {
        const Tile src = source(options_.zoom - d, x >> d, y >> d);
        if (!src) continue;
        if (!src->valid()) return 0.0;
        const unsigned mask = (1u << d) - 1u, qx = x & mask, qy = y & mask;
        const double share = 1.0 / static_cast<double>(1u << d);
        return bilinear(src->size, tx, ty, centres, [&](std::uint32_t i, std::uint32_t j) {
            return static_cast<double>(ancestorTexel(*src, qx, qy, share, i, j));
        });
    }
    return 0.0;
}

double TerrainTiles::heightAboveEllipsoidM(double latitudeRad, double longitudeRad) const {
    double tx, ty;
    mercatorTile(latitudeRad, longitudeRad, options_.zoom, tx, ty);
    const double n = static_cast<double>(1u << options_.zoom);
    const Tile t = tile(static_cast<unsigned>(std::floor(tx)), static_cast<unsigned>(std::clamp(std::floor(ty), 0.0, n - 1.0)));
    if (!t || !t->valid()) return 0.0;
    return sample(*t, tx, ty, options_.meshDimension == 0);
}

std::optional<double> TerrainTiles::knownHeightAboveEllipsoidM(double latitudeRad, double longitudeRad) const {
    double tx, ty;
    mercatorTile(latitudeRad, longitudeRad, options_.zoom, tx, ty);
    const double n = static_cast<double>(1u << options_.zoom);
    const Tile t = tile(static_cast<unsigned>(std::floor(tx)), static_cast<unsigned>(std::clamp(std::floor(ty), 0.0, n - 1.0)));
    if (!t || !t->valid()) return std::nullopt;
    return sample(*t, tx, ty, options_.meshDimension == 0);
}

namespace {

/// The tile coordinates (Mercator at `tiles` tiles a side, as mercatorTile gives them) of `n` points evenly spaced
/// along the great circle from one position to another, both ends included, each handed to `at(i, tx, ty)`.
/// Exactly - from the point's unit vector, x its longitude's atan2, y atanh(sin latitude) - at nodes every
/// kAlongNode points and at the far end; between them, the cubic Hermite through the two nodes' coordinates and
/// their rates along the circle. A great circle is analytic in Mercator, smooth on the scale of the earth: over a
/// span h the cubic is out by ~(h/R)^4/384 of a radian - 1e-15 rad over 32 points of a 2,000-point path, a
/// micrometre - for two transcendentals a node instead of two a point. Near the poles (where Mercator runs away),
/// each point exactly.
constexpr int kAlongNode = 32;

template <class F>
void mercatorAlong(double lat0, double lon0, double lat1, double lon1, int n, double tiles, F&& at) {
    constexpr double pi = 3.14159265358979323846;
    const double zMax = std::sin(1.4844222297453322); // (+-85.0511 deg, as mercatorTile clamps)
    const double c0 = std::cos(lat0), c1 = std::cos(lat1);
    const double a[3] = {c0 * std::cos(lon0), c0 * std::sin(lon0), std::sin(lat0)};
    const double b[3] = {c1 * std::cos(lon1), c1 * std::sin(lon1), std::sin(lat1)};
    const double cr[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    const double sinW = std::sqrt(cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]);
    const double omega = std::atan2(sinW, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
    const double k = 0.5 * tiles / pi; // (tile units a radian)
    struct Node {
        double tx, ty, dtx, dty, z;
    };
    auto exact = [&](const double* p, const double* t) { // a point's coordinates and their rates (t: dp per index)
        Node q;
        q.z = p[2];
        q.tx = (std::atan2(p[1], p[0]) / pi + 1.0) * 0.5 * tiles;
        if (q.tx < 0.0) q.tx += tiles;
        if (q.tx >= tiles) q.tx -= tiles;
        const double zc = std::clamp(p[2], -zMax, zMax);
        q.ty = (1.0 - std::atanh(zc) / pi) * 0.5 * tiles;
        const double rho2 = p[0] * p[0] + p[1] * p[1];
        q.dtx = rho2 > 0.0 ? k * (p[0] * t[1] - p[1] * t[0]) / rho2 : 0.0;
        q.dty = std::fabs(p[2]) < zMax ? -k * t[2] / (1.0 - p[2] * p[2]) : 0.0;
        return q;
    };
    if (n < 2 || !(sinW > 1e-15)) { // (one point, or the two ends together)
        const double t0[3] = {0, 0, 0};
        const Node q = exact(a, t0);
        for (int i = 0; i < n; ++i) at(i, q.tx, q.ty);
        return;
    }
    // the unit vector in the circle's plane at right angles to a, towards b; a point and its rate at index i
    const double cw = std::cos(omega);
    const double u[3] = {(b[0] - a[0] * cw) / sinW, (b[1] - a[1] * cw) / sinW, (b[2] - a[2] * cw) / sinW};
    const double delta = omega / static_cast<double>(n - 1);
    auto pointAt = [&](int i, double* p, double* t) {
        if (i == n - 1) { // (the far end exactly)
            for (int c = 0; c < 3; ++c) p[c] = b[c], t[c] = delta * (-a[c] * std::sin(omega) + u[c] * cw);
            return;
        }
        const double s = std::sin(delta * i), c = std::cos(delta * i);
        for (int m = 0; m < 3; ++m) p[m] = a[m] * c + u[m] * s, t[m] = delta * (-a[m] * s + u[m] * c);
    };
    double p[3], t[3];
    pointAt(0, p, t);
    Node n0 = exact(p, t);
    for (int i0 = 0; i0 < n - 1; i0 += kAlongNode) {
        const int i1 = std::min(i0 + kAlongNode, n - 1);
        pointAt(i1, p, t);
        const Node n1 = exact(p, t);
        if (std::max(std::fabs(n0.z), std::fabs(n1.z)) > 0.98) { // (near a pole: each point exactly)
            for (int i = i0; i < i1; ++i) {
                double pi_[3], ti[3];
                pointAt(i, pi_, ti);
                const Node q = exact(pi_, ti);
                at(i, q.tx, q.ty);
            }
        } else {
            double x1 = n1.tx; // (across the antimeridian: unwrapped)
            if (x1 - n0.tx > 0.5 * tiles) x1 -= tiles;
            else if (n0.tx - x1 > 0.5 * tiles) x1 += tiles;
            const double h = static_cast<double>(i1 - i0);
            at(i0, n0.tx, n0.ty);
            for (int i = i0 + 1; i < i1; ++i) {
                const double s = static_cast<double>(i - i0) / h, s2 = s * s, s3 = s2 * s;
                const double h00 = 2 * s3 - 3 * s2 + 1, h10 = s3 - 2 * s2 + s, h01 = -2 * s3 + 3 * s2, h11 = s3 - s2;
                double tx = h00 * n0.tx + h10 * h * n0.dtx + h01 * x1 + h11 * h * n1.dtx;
                const double ty = h00 * n0.ty + h10 * h * n0.dty + h01 * n1.ty + h11 * h * n1.dty;
                if (tx < 0.0) tx += tiles;
                if (tx >= tiles) tx -= tiles;
                at(i, tx, ty);
            }
        }
        n0 = n1;
    }
    at(n - 1, n0.tx, n0.ty);
}

} // namespace

void TerrainTiles::profile(double lat0, double lon0, double lat1, double lon1, int n, double, double* out) const {
    const unsigned z = options_.zoom;
    const double tiles = static_cast<double>(1u << z);
    const bool centres = options_.meshDimension == 0;
    if (!options_.offline || fetch_) { // (this level's own tiles, fetched where missing: a tile at a time)
        unsigned cx = ~0u, cy = ~0u;
        Tile t;
        mercatorAlong(lat0, lon0, lat1, lon1, n, tiles, [&](int i, double tx, double ty) {
            ty = std::clamp(ty, 0.0, tiles - 1e-9);
            const auto x = static_cast<unsigned>(std::floor(tx)), y = static_cast<unsigned>(std::floor(ty));
            if (x != cx || y != cy) t = tile(x, y), cx = x, cy = y;
            out[i] = t && t->valid() ? sample(*t, tx, ty, centres) : 0.0;
        });
        return;
    }
    // Offline: the deepest level on disk at each point - the source this level's tile there is made from (load(),
    // fromAncestor()) - read directly, a source tile at a time
    unsigned sd = ~0u, sx = ~0u, sy = ~0u; // (the source tile in use: d levels up, at sx, sy)
    Tile t;
    mercatorAlong(lat0, lon0, lat1, lon1, n, tiles, [&](int i, double tx, double ty) {
        ty = std::clamp(ty, 0.0, tiles - 1e-9);
        const auto x = static_cast<unsigned>(std::floor(tx)), y = static_cast<unsigned>(std::floor(ty));
        if (!(t && (x >> sd) == sx && (y >> sd) == sy)) { // (left the source tile: the deepest one holding this point)
            t = nullptr;
            for (unsigned d = 0; d <= z && !t; ++d) {
                Tile c = source(z - d, x >> d, y >> d);
                if (c && c->valid()) t = std::move(c), sd = d, sx = x >> d, sy = y >> d;
            }
        }
        if (!t) {
            out[i] = 0.0;
            return;
        }
        const double scale = 1.0 / static_cast<double>(1u << sd); // (the point at the source's level)
        if (sd == 0) {
            out[i] = sample(*t, tx, ty, centres); // (this level on disk: as heightAboveEllipsoidM reads it)
            return;
        }
        // an ancestor: at its texel centres, held to its edge texels as fromAncestor() holds a tile made from it - the
        // ground the vehicles stand on, not extrapolated half a coarse texel beyond it
        const double size = static_cast<double>(t->size), hi = size - 1.0;
        const double fx = std::clamp((tx * scale - std::floor(tx * scale)) * size - 0.5, 0.0, hi);
        const double fy = std::clamp((ty * scale - std::floor(ty * scale)) * size - 0.5, 0.0, hi);
        const auto x0 = static_cast<std::uint32_t>(std::min(std::floor(fx), hi - 1.0));
        const auto y0 = static_cast<std::uint32_t>(std::min(std::floor(fy), hi - 1.0));
        const double ax = fx - x0, ay = fy - y0;
        auto at = [&](std::uint32_t a, std::uint32_t b) { return static_cast<double>(t->heights[static_cast<std::size_t>(b) * t->size + a]); };
        out[i] = (at(x0, y0) * (1 - ax) + at(x0 + 1, y0) * ax) * (1 - ay) + (at(x0, y0 + 1) * (1 - ax) + at(x0 + 1, y0 + 1) * ax) * ay;
    });
}

double TerrainTiles::resolutionM() const noexcept {
    const double across = options_.meshDimension >= 2 ? options_.meshDimension - 1.0 : 256.0; // (resampled, or the raster's pixels)
    return 40075016.686 / static_cast<double>(1u << options_.zoom) / across;
}

std::optional<double> TerrainTiles::cachedHeightAboveEllipsoidM(double latitudeRad, double longitudeRad) {
    double tx, ty;
    mercatorTile(latitudeRad, longitudeRad, options_.zoom, tx, ty);
    const double n = static_cast<double>(1u << options_.zoom);
    const Key key{static_cast<unsigned>(std::floor(tx)), static_cast<unsigned>(std::clamp(std::floor(ty), 0.0, n - 1.0))};
    Tile t;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (auto it = cache_.find(key); it != cache_.end()) {
            lru_.splice(lru_.begin(), lru_, it->second.second);
            t = it->second.first;
        }
    }
    if (!t) {
        requestAround(latitudeRad, longitudeRad);
        return std::nullopt;
    }
    if (!t->valid()) return 0.0;
    return sample(*t, tx, ty, options_.meshDimension == 0);
}

void TerrainTiles::prefetch(double latitudeRad, double longitudeRad, double radiusM, unsigned threads) {
    double tx, ty;
    mercatorTile(latitudeRad, longitudeRad, options_.zoom, tx, ty);
    const int n = static_cast<int>(1u << options_.zoom);
    const double metresPerTile = 2.0 * kPi * 6378137.0 * std::cos(latitudeRad) / n;
    const int r = static_cast<int>(std::ceil(radiusM / metresPerTile));
    const int cx = static_cast<int>(tx), cy = static_cast<int>(ty);
    std::vector<Key> keys;
    for (int dy = -r; dy <= r; ++dy)
        for (int dx = -r; dx <= r; ++dx) {
            const int y = cy + dy;
            if (y < 0 || y >= n) continue;
            keys.push_back(Key{static_cast<unsigned>(((cx + dx) % n + n) % n), static_cast<unsigned>(y)});
        }
    std::atomic<std::size_t> next{0};
    auto worker = [&] {
        for (std::size_t i = next.fetch_add(1); i < keys.size(); i = next.fetch_add(1)) tile(keys[i].x, keys[i].y);
    };
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < std::max(1u, std::min<unsigned>(threads, static_cast<unsigned>(keys.size()))); ++t) pool.emplace_back(worker);
    for (auto& t : pool) t.join();
    LOG_DEBUG("io") << "prefetched " << keys.size() << " elevation tiles at zoom " << options_.zoom << " (" << failures_.load() << " failures)";
}

void TerrainTiles::requestAround(double latitudeRad, double longitudeRad) {
    double tx, ty;
    mercatorTile(latitudeRad, longitudeRad, options_.zoom, tx, ty);
    const int n = static_cast<int>(1u << options_.zoom);
    const int cx = static_cast<int>(tx), cy = static_cast<int>(ty);
    bool any = false;
    {
        std::lock_guard<std::mutex> cacheLock(mutex_);
        std::lock_guard<std::mutex> queueLock(queueMutex_);
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                const int y = cy + dy;
                if (y < 0 || y >= n) continue;
                const Key k{static_cast<unsigned>(((cx + dx) % n + n) % n), static_cast<unsigned>(y)};
                const std::uint64_t id = (static_cast<std::uint64_t>(k.x) << 32) | k.y;
                if (cache_.count(k) || queued_.count(id)) continue;
                queued_.insert(id);
                queue_.push_back(k);
                any = true;
            }
    }
    if (any) {
        ensureLoaders();
        queueCv_.notify_all();
    }
}

void TerrainTiles::ensureLoaders() {
    std::lock_guard<std::mutex> lock(queueMutex_);
    if (!loaders_.empty()) return;
    for (unsigned i = 0; i < std::max(1u, options_.loaderThreads); ++i) loaders_.emplace_back([this] { loaderLoop(); });
}

void TerrainTiles::loaderLoop() {
    for (;;) {
        Key k;
        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            queueCv_.wait(lock, [&] { return stopLoaders_.load() || !queue_.empty(); });
            if (stopLoaders_.load()) return;
            k = queue_.front();
            queue_.pop_front();
        }
        tile(k.x, k.y);
        std::lock_guard<std::mutex> lock(queueMutex_);
        queued_.erase((static_cast<std::uint64_t>(k.x) << 32) | k.y);
    }
}

std::size_t TerrainTiles::cachedTiles() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cache_.size();
}

} // namespace fsim::io
