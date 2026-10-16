#include "foundation/core/HeightGrid.h"
#include "world/Terrain.h"

#include "foundation/core/Log.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace fsim::world {

namespace {

float gSeaFloor = 0.0f;

// (the sea, at or below 0 m, at the sea floor's depth; land above it as it is)
float terrariumHeight(float h) noexcept { return h > 0.0f ? h : gSeaFloor; }
float terrarium(const vsg::ubvec4& p) noexcept {
    return terrariumHeight(static_cast<float>(p.r) * 256.0f + static_cast<float>(p.g) + static_cast<float>(p.b) / 256.0f - 32768.0f);
}
float terrarium(const vsg::ubvec3& p) noexcept {
    return terrariumHeight(static_cast<float>(p.r) * 256.0f + static_cast<float>(p.g) + static_cast<float>(p.b) / 256.0f - 32768.0f);
}

// The sea floor (setSeaFloorDepth) sloped down from the shore rather than a step: the sea texel next to land 1 m down,
// then 4 m deeper a texel to the floor. The mesh joins a land vertex and a sea vertex by a straight line; to a floor
// 25 m down, the line stayed under sea level for nine tenths of the way up a beach, and the sea - tested at its surface
// - covered the sand that far inland (30 m at Ocean Beach, seen from 3 km). Chamfer distances within the tile.
void slopeSeaFloor(std::vector<float>& h, std::uint32_t w, std::uint32_t ht) {
    if (gSeaFloor >= 0.0f || w < 2 || ht < 2) return;
    const float far = 1e9f;
    std::vector<float> d(h.size());
    bool anyLand = false, anySea = false;
    for (std::size_t i = 0; i < h.size(); ++i) {
        const bool sea = h[i] <= 0.0f; // (the land above 0 m: idempotent - an upsampled tile, sloped already, comes back)
        d[i] = sea ? far : 0.0f;
        anyLand = anyLand || !sea;
        anySea = anySea || sea;
    }
    if (!anyLand || !anySea) return;
    const auto at = [&](std::int64_t x, std::int64_t y) -> float& { return d[static_cast<std::size_t>(y) * w + static_cast<std::size_t>(x)]; };
    const std::int64_t W = w, H = ht;
    for (std::int64_t y = 0; y < H; ++y)
        for (std::int64_t x = 0; x < W; ++x) {
            float& v = at(x, y);
            if (x > 0) v = std::min(v, at(x - 1, y) + 1.0f);
            if (y > 0) v = std::min(v, at(x, y - 1) + 1.0f);
            if (x > 0 && y > 0) v = std::min(v, at(x - 1, y - 1) + 1.41421f);
            if (x + 1 < W && y > 0) v = std::min(v, at(x + 1, y - 1) + 1.41421f);
        }
    for (std::int64_t y = H - 1; y >= 0; --y)
        for (std::int64_t x = W - 1; x >= 0; --x) {
            float& v = at(x, y);
            if (x + 1 < W) v = std::min(v, at(x + 1, y) + 1.0f);
            if (y + 1 < H) v = std::min(v, at(x, y + 1) + 1.0f);
            if (x + 1 < W && y + 1 < H) v = std::min(v, at(x + 1, y + 1) + 1.41421f);
            if (x > 0 && y + 1 < H) v = std::min(v, at(x - 1, y + 1) + 1.41421f);
        }
    for (std::size_t i = 0; i < h.size(); ++i)
        if (d[i] > 0.0f) h[i] = std::max(gSeaFloor, -1.0f - 4.0f * (d[i] - 1.0f));
}

// Resample to the mesh's vertex grid: output texel k holds the height at tile
// fraction k / (n - 1), which is where vsg::tile places vertex k (u = c / (cols - 1)).
// Sampling the source bilinearly at those positions makes both tiles of a
// shared edge (and a tile and its upsampled children) agree on the edge
// heights, so LOD transitions and tile seams do not open cracks on cliffs.
// (Nearest-pixel decimation put vertex k at pixel 4k: the edge vertices of
// neighbouring tiles then stood 3 pixels apart, 15 m at level 15 - tall fins
// on a 60-degree wall.)
template <typename Array, typename Fn>
vsg::ref_ptr<vsg::floatArray2D> convert(const Array& srcArray, std::uint32_t maxDim, Fn fn) {
    const std::uint32_t w = srcArray.width(), h = srcArray.height();
    const std::uint32_t ow = (maxDim > 0 && w > maxDim) ? maxDim : w, oh = (maxDim > 0 && h > maxDim) ? maxDim : h;

    std::vector<float> decoded(static_cast<std::size_t>(w) * h);
    for (std::uint32_t y = 0; y < h; ++y)
        for (std::uint32_t x = 0; x < w; ++x) decoded[static_cast<std::size_t>(y) * w + x] = fn(srcArray.at(x, y));
    slopeSeaFloor(decoded, w, h);

    // The filtering and resampling live in core so that io::TerrainTiles - which
    // answers the camera's "how high is the ground here?" - can build the same
    // surface from the same tiles. Two implementations of this drifted apart
    // once already and the camera flew into a hill it could see.
    std::vector<float> resampled;
    const std::vector<float>* grid = &decoded;
    if (w == h && ow < w) {
        resampled = core::resampleHeightGrid(decoded.data(), w, ow);
        grid = &resampled;
    }

    auto out = vsg::floatArray2D::create(ow, oh, vsg::Data::Properties{VK_FORMAT_R32_SFLOAT});
    out->properties.origin = srcArray.properties.origin;
    for (std::uint32_t y = 0; y < oh; ++y)
        for (std::uint32_t x = 0; x < ow; ++x) out->set(x, y, (*grid)[static_cast<std::size_t>(y) * ow + x]);
    return out;
}

} // namespace

void setSeaFloorDepth(float metres) { gSeaFloor = std::min(metres, 0.0f); }
float seaFloorDepth() { return gSeaFloor; }

vsg::ref_ptr<vsg::floatArray2D> decodeElevation(vsg::ref_ptr<vsg::Data> image, ElevationEncoding encoding,
                                                std::uint32_t maxDimension) {
    if (!image) return {};
    // Already heights (a synthesised tile from ElevationUpsampler): pass through, downsampled if asked.
    if (auto f = image.cast<vsg::floatArray2D>()) {
        if (maxDimension == 0 || f->width() <= maxDimension) return f;
        return convert(*f, maxDimension, [](float v) { return v; });
    }
    switch (encoding) {
    case ElevationEncoding::Float:
        if (auto f = image.cast<vsg::floatArray2D>()) {
            if (maxDimension == 0 || f->width() <= maxDimension) return f;
            return convert(*f, maxDimension, [](float v) { return v; });
        }
        if (auto s = image.cast<vsg::ushortArray2D>())
            return convert(*s, maxDimension, [](std::uint16_t v) { return static_cast<float>(v); });
        break;
    case ElevationEncoding::Terrarium:
        if (auto rgba = image.cast<vsg::ubvec4Array2D>()) return convert(*rgba, maxDimension, [](const vsg::ubvec4& p) { return terrarium(p); });
        if (auto rgb = image.cast<vsg::ubvec3Array2D>()) return convert(*rgb, maxDimension, [](const vsg::ubvec3& p) { return terrarium(p); });
        break;
    }
    LOG_WARN("world") << "elevation tile has an unexpected pixel format (" << image->className() << ")";
    return {};
}

} // namespace fsim::world
