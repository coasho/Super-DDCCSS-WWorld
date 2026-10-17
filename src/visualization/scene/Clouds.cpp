#include "visualization/scene/Clouds.h"

#include "foundation/core/Log.h"
#include "foundation/io/TerrainTiles.h"
#include "foundation/platform/Threads.h"
#include "visualization/scene/Sky.h"
#include "visualization/scene/Terrain.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

namespace fsim::world {

// The shaders' uniform block, member for member (assets/clouds/shaders/fsim/params.glsl; std140: all of it 16-byte
// columns).
struct Clouds::Params {
    vsg::mat4 inverseProjection, projection, inverseView, view, reprojection, viewReprojection;
    vsg::mat4 shadow[3], inverseShadow[3], shadowReprojection[3];
    vsg::vec4 camera, altitudeCorrection, sun, screen, alloc, frame, nearFar;
    vsg::vec4 shadowIntervals[3];
    vsg::vec4 shadowTexel, weather, shapeRepeat, shapeOffset, shapeDetailRepeat, shapeDetailOffset, turbulence;
    vsg::vec4 minLayerHeights, maxLayerHeights, minIntervalHeights, maxIntervalHeights;
    vsg::vec4 densityScales, shapeAmounts, shapeDetailAmounts, weatherExponents, shapeAlteringBiases, coverageFilterWidths;
    vsg::vec4 heights, shadowLayerMask;
    vsg::vec4 expTerms, exponents, linearTerms, constantTerms;
    vsg::vec4 march, march2, secondary, light, medium, shadowMarch, shadowMarch2, ground, global, night;
    vsg::vec4 ocean, oceanSpectrum, oceanSpectrum2, oceanFoam, oceanEye, oceanRadii, oceanU, oceanV;
    vsg::vec4 cascade[3], cascadePhase[3], oceanHaze, oceanLight, oceanGrid, shore0, shoreEast, shoreNorth;
};
static_assert(sizeof(Clouds::Params) % 16 == 0);

namespace {

double smoothstep(double e0, double e1, double x) {
    const double t = std::clamp((x - e0) / (e1 - e0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// takram's "high" quality preset (qualityPresets.ts) and their defaults (CloudsMaterial.ts, uniforms.ts,
// CloudsEffect.ts, CloudLayers.ts)
constexpr std::uint32_t kShadowMap = 512, kCascades = 3;
constexpr double kShadowMaxFar = 1e5, kSplitLambda = 0.6;      // their stories' maxFar; CloudsEffect's splitLambda
constexpr double kBottomRadius = 6360000.0;                    // AtmosphereParameters.bottomRadius
constexpr double kWeatherRepeat = 100.0, kShapeRepeat = 0.0003, kDetailRepeat = 0.006, kTurbulenceRepeat = 20.0;
// where the global map's cover takes over from the world's (along the ground from the point under the eye, m), and
// the eye's heights over which it takes over everywhere
constexpr double kGlobalFrom = 150000.0, kGlobalTo = 300000.0, kGlobalHighFrom = 30000.0, kGlobalHighTo = 100000.0;
// the reach of the global map's mean round the eye (m along the ground), and how far the eye goes before it is taken
// again
constexpr double kAroundRadius = 150000.0, kAroundMoved = 10000.0;

// the sea (assets/clouds/shaders/ocean): its maps' size, its cascades' tiles (m; none a simple ratio of another, so
// their patterns never line up) and where each one's band of the spectrum starts - at six waves to the tile of its
// own, the waves longer than that left to the cascade before it
constexpr std::uint32_t kSeaMap = 256, kSeaLevels = 9, kSeaCascades = 3, kSeaSpectra = 4;
constexpr double kSeaTiles[kSeaCascades] = {1987.0, 409.0, 71.0};
constexpr double kSeaBandStart = 6.0;
constexpr double kSeaPeriod = 1000.0;       // s: the waves' clock wraps (ocean/ocean.glsl's OCEAN_PERIOD)
constexpr double kSeaWindDefault = 7.0;     // m/s: the open ocean's mean wind, with no weather that says otherwise
constexpr double kSeaFetch = 300000.0;      // m: the open ocean's
// the dimensionless fetch (g F / U^2) past which the sea is fully developed: JONSWAP's fetch laws grow its waves without
// end, and past it they no longer should. 12,000 gives Pierson & Moskowitz's significant height for 5 - 15 m/s within
// 3 % (0.6 m at 5 m/s, 1.2 m at 7, 2.4 m at 10, 5.4 m at 15)
constexpr double kSeaFullyDeveloped = 12000.0;
constexpr double kSeaReanchor = 500000.0;   // m: the eye this far from the plane's anchor moves it
constexpr float kSeaGridMargin = 0.12f;      // the sea's grid beyond the screen, each side (0..1 of it)
// the shore round the eye (ShoreMap): texels and their spacing (m), the eye's height above which it is not made, how far
// the eye moves before it is made again, and the distance one step of its 8-bit code stands for (m)
constexpr std::uint32_t kShoreSize = 512;
constexpr double kShoreSpacing = 25.0, kShoreMaxAltitude = 8000.0, kShoreMoved = 2500.0, kShoreStep = 2.0;
constexpr double kSeaWindLag = 300.0;       // s: how long the sea takes to follow a change of wind

// the cover each coverage gives (tools/data/cloud_map/coverage_curve.py; also in fsim/coverage.glsl)
constexpr double kCurveCover[] = {0.0,   0.006, 0.016, 0.034, 0.059, 0.093, 0.136, 0.187, 0.249, 0.321, 0.398,
                                  0.481, 0.569, 0.656, 0.739, 0.824, 0.904, 0.960, 0.985, 0.995, 1.0};
constexpr double kCurveCoverage[] = {0.10, 0.15,  0.175, 0.2,   0.225, 0.25, 0.275, 0.3,   0.325, 0.35, 0.375,
                                     0.4,  0.425, 0.45,  0.475, 0.5,   0.525, 0.55, 0.575, 0.6,   0.65};

// The global cloud map's mean cover within `radius` (m along the ground) of a point: its texels (the map's north row
// first, longitude from -180 at its left edge, as fsim/coverage.glsl reads it) turned into cover as Skybolt turns them
// (map * 1.3 - 0.001), every other texel each way.
double mapCoverAround(const vsg::Data& map, double latDeg, double lonDeg, double radius) {
    const auto w = static_cast<int>(map.width()), h = static_cast<int>(map.height());
    if (w <= 0 || h <= 0 || map.valueSize() != 1) return 0.0;
    const auto* texels = static_cast<const std::uint8_t*>(map.dataPointer());
    const double perTexel = 2.0 * 3.141592653589793 * 6371000.0 / w; // m along the equator
    const int ry = std::max(1, static_cast<int>(radius / perTexel));
    const int rx = std::min(w / 2, static_cast<int>(ry / std::max(std::cos(vsg::radians(latDeg)), 0.05)));
    const int cx = static_cast<int>((lonDeg / 360.0 + 0.5) * w), cy = static_cast<int>((0.5 - latDeg / 180.0) * h);
    double sum = 0.0;
    int n = 0;
    for (int dy = -ry; dy <= ry; dy += 2)
        for (int dx = -rx; dx <= rx; dx += 2) {
            const int y = std::clamp(cy + dy, 0, h - 1), x = ((cx + dx) % w + w) % w;
            sum += std::clamp(texels[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] / 255.0 * 1.3 - 0.001, 0.0, 1.0);
            ++n;
        }
    return n > 0 ? sum / n : 0.0;
}

// The shaders: a file, its #include "..." lines replaced by the files they name (relative to the shaders' folder).
bool loadShaderSource(const std::filesystem::path& root, const std::string& name, std::string& out, int depth = 0) {
    std::ifstream in(root / name, std::ios::binary);
    if (!in || depth > 8) return false;
    std::stringstream ss;
    ss << in.rdbuf();
    static const std::regex include(R"re(^\s*#include\s+"([^"]+)"\s*$)re");
    std::string line;
    std::istringstream lines(ss.str());
    while (std::getline(lines, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::smatch m;
        if (std::regex_match(line, m, include)) {
            if (!loadShaderSource(root, m[1].str(), out, depth + 1)) return false;
        } else {
            out += line;
            out += '\n';
        }
    }
    return true;
}

vsg::ref_ptr<vsg::ShaderStage> shaderFile(const std::filesystem::path& root, const std::string& name, VkShaderStageFlagBits kind, int mode = -1) {
    std::string src;
    if (!loadShaderSource(root, name, src)) {
        LOG_WARN("world") << "clouds: cannot read " << (root / name).string();
        return {};
    }
    if (mode >= 0) {
        const auto at = src.find("#version 450\n");
        if (at == std::string::npos) return {};
        src.insert(at + 13, "#define MODE " + std::to_string(mode) + "\n");
    }
    return vsg::ShaderStage::create(kind, "main", src);
}

bool readBytes(const std::filesystem::path& path, std::vector<std::uint8_t>& bytes, std::size_t expected) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    bytes.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return bytes.size() == expected;
}

// takram's textures, as they load them: R8 3D data (DataTextureLoader), float16 RGBA tables
// (PrecomputedTexturesLoader), PNGs through three's TextureLoader (flipY: the image's last row at v = 0)
vsg::ref_ptr<vsg::Data> r8Volume(const std::filesystem::path& path, std::uint32_t w, std::uint32_t h, std::uint32_t d) {
    std::vector<std::uint8_t> bytes;
    if (!readBytes(path, bytes, std::size_t{w} * h * d)) return {};
    auto data = vsg::ubyteArray3D::create(w, h, d, vsg::Data::Properties{VK_FORMAT_R8_UNORM});
    std::memcpy(data->dataPointer(), bytes.data(), bytes.size());
    return data;
}

// The same with its mip levels, each the average of the eight texels under it (wrapping, as the texture repeats):
// the shaders read the noise at the level a pixel's footprint asks for, as a fragment shader's implicit derivatives
// would (takram's has none; from far off it aliased into separate, flickering puffs).
vsg::ref_ptr<vsg::Data> r8VolumeMips(const std::filesystem::path& path, std::uint32_t n) {
    std::vector<std::uint8_t> level;
    if (!readBytes(path, level, std::size_t{n} * n * n)) return {};
    std::vector<std::uint8_t> all = level;
    std::uint8_t levels = 1;
    for (std::uint32_t size = n; size > 1; size /= 2, ++levels) {
        const std::uint32_t half = size / 2;
        std::vector<std::uint8_t> next(std::size_t{half} * half * half);
        for (std::uint32_t z = 0; z < half; ++z)
            for (std::uint32_t y = 0; y < half; ++y)
                for (std::uint32_t x = 0; x < half; ++x) {
                    unsigned sum = 0;
                    for (std::uint32_t k = 0; k < 8; ++k) {
                        const std::uint32_t sx = 2 * x + (k & 1), sy = 2 * y + ((k >> 1) & 1), sz = 2 * z + (k >> 2);
                        sum += level[(std::size_t{sz} * size + sy) * size + sx];
                    }
                    next[(std::size_t{z} * half + y) * half + x] = static_cast<std::uint8_t>((sum + 4) / 8);
                }
        all.insert(all.end(), next.begin(), next.end());
        level.swap(next);
    }
    auto storage = vsg::ubyteArray::create(static_cast<std::uint32_t>(all.size()));
    std::memcpy(storage->dataPointer(), all.data(), all.size());
    vsg::Data::Properties properties{VK_FORMAT_R8_UNORM};
    properties.mipLevels = levels;
    return vsg::ubyteArray3D::create(storage, 0, 1, n, n, n, properties);
}

vsg::ref_ptr<vsg::Data> halfTable(const std::filesystem::path& path, std::uint32_t w, std::uint32_t h, std::uint32_t d) {
    std::vector<std::uint8_t> bytes;
    if (!readBytes(path, bytes, std::size_t{w} * h * d * 8)) return {};
    vsg::ref_ptr<vsg::Data> data;
    if (d == 1)
        data = vsg::usvec4Array2D::create(w, h, vsg::Data::Properties{VK_FORMAT_R16G16B16A16_SFLOAT});
    else
        data = vsg::usvec4Array3D::create(w, h, d, vsg::Data::Properties{VK_FORMAT_R16G16B16A16_SFLOAT});
    std::memcpy(data->dataPointer(), bytes.data(), bytes.size());
    return data;
}

vsg::ref_ptr<vsg::Data> rgbaPng(const std::filesystem::path& path) {
    std::vector<std::uint8_t> pixels;
    std::uint32_t w = 0, h = 0;
    if (!io::readRgbaPng(path, pixels, w, h)) return {};
    auto data = vsg::ubvec4Array2D::create(w, h, vsg::Data::Properties{VK_FORMAT_R8G8B8A8_UNORM});
    const std::size_t row = std::size_t{w} * 4;
    auto* dst = static_cast<std::uint8_t*>(data->dataPointer());
    for (std::uint32_t y = 0; y < h; ++y) std::memcpy(dst + row * y, pixels.data() + row * (h - 1 - y), row);
    return data;
}

vsg::ref_ptr<vsg::Sampler> sampler(VkFilter filter, VkSamplerAddressMode address, float maxLod) {
    auto s = vsg::Sampler::create();
    s->minFilter = s->magFilter = filter;
    s->mipmapMode = filter == VK_FILTER_LINEAR ? VK_SAMPLER_MIPMAP_MODE_LINEAR : VK_SAMPLER_MIPMAP_MODE_NEAREST;
    s->addressModeU = s->addressModeV = s->addressModeW = address;
    s->maxLod = maxLod;
    return s;
}

vsg::ref_ptr<vsg::Image> makeImage(VkFormat format, std::uint32_t w, std::uint32_t h, std::uint32_t layers) {
    auto image = vsg::Image::create();
    image->imageType = VK_IMAGE_TYPE_2D;
    image->format = format;
    image->extent = VkExtent3D{w, h, 1};
    image->mipLevels = 1;
    image->arrayLayers = layers;
    image->samples = VK_SAMPLE_COUNT_1_BIT;
    image->tiling = VK_IMAGE_TILING_OPTIMAL;
    image->usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    return image;
}

vsg::ref_ptr<vsg::ImageView> viewOf(vsg::ref_ptr<vsg::Image> image) {
    auto view = vsg::ImageView::create(image, VK_IMAGE_ASPECT_COLOR_BIT);
    view->viewType = image->arrayLayers > 1 ? VK_IMAGE_VIEW_TYPE_2D_ARRAY : VK_IMAGE_VIEW_TYPE_2D;
    view->subresourceRange = VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, image->mipLevels, 0, image->arrayLayers};
    return view;
}

// One mip level of an image, as an array (the sea's maps: written level by level).
vsg::ref_ptr<vsg::ImageView> levelOf(vsg::ref_ptr<vsg::Image> image, std::uint32_t level) {
    auto view = vsg::ImageView::create(image, VK_IMAGE_ASPECT_COLOR_BIT);
    view->viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
    view->subresourceRange = VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, level, 1, 0, image->arrayLayers};
    return view;
}

// The signed distance from each texel of a square round a point on the ellipsoid to the shore (where the ground query
// says the ground is above 0 m): positive over the sea, negative on land, as 128 + d / kShoreStep (+-254 m) - chamfer
// distance transforms both ways, smoothed, so the zero line is a smooth coast.
std::vector<std::uint8_t> shoreDistances(const std::function<std::optional<double>(double, double)>& query, const vsg::dvec3& centre,
                                         const vsg::dvec3& east, const vsg::dvec3& north, double a, double b, bool& anyLand) {
    const std::uint32_t n = kShoreSize;
    auto ellipsoid = vsg::EllipsoidModel::create(a, b);
    std::vector<float> d(static_cast<std::size_t>(n) * n, 1e9f);
    std::vector<std::uint8_t> isLand(d.size(), 0);
    anyLand = false;
    for (std::uint32_t j = 0; j < n; ++j)
        for (std::uint32_t i = 0; i < n; ++i) {
            const double e = (i + 0.5 - n * 0.5) * kShoreSpacing, no = (j + 0.5 - n * 0.5) * kShoreSpacing;
            const vsg::dvec3 lla = ellipsoid->convertECEFToLatLongAltitude(centre + east * e + north * no);
            const auto h = query(vsg::radians(lla.x), vsg::radians(lla.y));
            if (h && *h > 0.002) {
                d[static_cast<std::size_t>(j) * n + i] = 0.0f;
                isLand[static_cast<std::size_t>(j) * n + i] = 1;
                anyLand = true;
            }
        }
    const std::int64_t N = n;
    // the chamfer transform of a field (0 at its sources): two passes
    const auto chamfer = [&](std::vector<float>& f) {
        const auto at = [&](std::int64_t x, std::int64_t y) -> float& { return f[static_cast<std::size_t>(y) * n + static_cast<std::size_t>(x)]; };
        for (std::int64_t y = 0; y < N; ++y)
            for (std::int64_t x = 0; x < N; ++x) {
                float& v = at(x, y);
                if (x > 0) v = std::min(v, at(x - 1, y) + 1.0f);
                if (y > 0) v = std::min(v, at(x, y - 1) + 1.0f);
                if (x > 0 && y > 0) v = std::min(v, at(x - 1, y - 1) + 1.41421f);
                if (x + 1 < N && y > 0) v = std::min(v, at(x + 1, y - 1) + 1.41421f);
            }
        for (std::int64_t y = N - 1; y >= 0; --y)
            for (std::int64_t x = N - 1; x >= 0; --x) {
                float& v = at(x, y);
                if (x + 1 < N) v = std::min(v, at(x + 1, y) + 1.0f);
                if (y + 1 < N) v = std::min(v, at(x, y + 1) + 1.0f);
                if (x + 1 < N && y + 1 < N) v = std::min(v, at(x + 1, y + 1) + 1.41421f);
                if (x > 0 && y + 1 < N) v = std::min(v, at(x - 1, y + 1) + 1.41421f);
            }
    };
    chamfer(d); // (over the sea: to the land)
    std::vector<float> toSea(d.size());
    for (std::size_t k = 0; k < d.size(); ++k) toSea[k] = isLand[k] ? 1e9f : 0.0f;
    chamfer(toSea); // (on land: to the sea)
    // signed, in metres, the coast half a texel from each (+-300 m at most: farther is far)
    for (std::size_t k = 0; k < d.size(); ++k)
        d[k] = static_cast<float>(std::clamp(isLand[k] ? -(toSea[k] - 0.5) * kShoreSpacing : (d[k] - 0.5) * kShoreSpacing, -300.0, 300.0));
    const auto at = [&](std::int64_t x, std::int64_t y) -> float& { return d[static_cast<std::size_t>(y) * n + static_cast<std::size_t>(x)]; };
    // smoothed (two passes of a 5 x 5 box, separable): the land is the elevation raster's, in steps of its texels, and
    // the surf's lines along the distance's contours traced every step
    std::vector<float> tmp(d.size());
    for (int pass = 0; pass < 2; ++pass)
        for (int axis = 0; axis < 2; ++axis) {
            for (std::int64_t y = 0; y < N; ++y)
                for (std::int64_t x = 0; x < N; ++x) {
                    float sum = 0.0f;
                    for (std::int64_t o = -2; o <= 2; ++o) {
                        const std::int64_t xx = std::clamp(axis == 0 ? x + o : x, std::int64_t{0}, N - 1);
                        const std::int64_t yy = std::clamp(axis == 1 ? y + o : y, std::int64_t{0}, N - 1);
                        sum += at(xx, yy);
                    }
                    tmp[static_cast<std::size_t>(y) * n + static_cast<std::size_t>(x)] = sum / 5.0f;
                }
            d.swap(tmp);
        }
    std::vector<std::uint8_t> out(d.size());
    for (std::size_t k = 0; k < d.size(); ++k)
        out[k] = static_cast<std::uint8_t>(std::clamp(std::round(128.0 + d[k] / kShoreStep), 0.0, 255.0));
    return out;
}

// The Stockham FFT's butterflies for a map of `size` (GodotOceanWaves' fft_butterfly.glsl, worked out once here):
// per stage and column the two indices it reads (as float bits) and its twiddle factor.
vsg::ref_ptr<vsg::vec4Array> fftButterflies(std::uint32_t size) {
    std::uint32_t stages = 0;
    while ((1u << stages) < size) ++stages;
    auto b = vsg::vec4Array::create(stages * size);
    const auto bits = [](std::uint32_t u) {
        float f;
        std::memcpy(&f, &u, sizeof f);
        return f;
    };
    for (std::uint32_t stage = 0; stage < stages; ++stage)
        for (std::uint32_t col = 0; col < size / 2; ++col) {
            const std::uint32_t stride = 1u << stage, mid = size >> (stage + 1);
            const std::uint32_t i = col >> stage, j = col % stride;
            const double angle = 3.141592653589793 / stride * j;
            const float c = static_cast<float>(std::cos(angle)), s = static_cast<float>(std::sin(angle));
            const std::uint32_t r0 = stride * i + j, r1 = stride * (i + mid) + j;
            const std::uint32_t w0 = stride * (2 * i) + j, w1 = stride * (2 * i + 1) + j;
            b->at(stage * size + w0) = vsg::vec4(bits(r0), bits(r1), c, s);
            b->at(stage * size + w1) = vsg::vec4(bits(r0), bits(r1), -c, -s);
        }
    return b;
}

vsg::ref_ptr<vsg::PipelineBarrier> computeDone(VkPipelineStageFlags before, VkAccessFlags access) {
    auto b = vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, before, 0);
    b->memoryBarriers.push_back(vsg::MemoryBarrier::create(VK_ACCESS_SHADER_WRITE_BIT, access));
    return b;
}

// A descriptor and the binding it is at.
struct Bound {
    std::uint32_t binding;
    VkDescriptorType type;
    vsg::ref_ptr<vsg::Descriptor> descriptor;
};

// three.js's Matrix4.lookAt (the rotation: z from target to eye) and makeOrthographic, in VSG's column-major dmat4
vsg::dmat4 threeLookAt(const vsg::dvec3& eye, const vsg::dvec3& target, vsg::dvec3 up) {
    vsg::dvec3 z = eye - target;
    if (vsg::length(z) == 0.0) z.z = 1.0;
    z = vsg::normalize(z);
    vsg::dvec3 x = vsg::cross(up, z);
    if (vsg::length(x) < 1e-12) {
        if (std::abs(up.z) == 1.0) z.x += 0.0001;
        else z.z += 0.0001;
        z = vsg::normalize(z);
        x = vsg::cross(up, z);
    }
    x = vsg::normalize(x);
    const vsg::dvec3 y = vsg::cross(z, x);
    return vsg::dmat4(x.x, x.y, x.z, 0.0, y.x, y.y, y.z, 0.0, z.x, z.y, z.z, 0.0, 0.0, 0.0, 0.0, 1.0);
}

vsg::dmat4 threeOrthographic(double left, double right, double top, double bottom, double near, double far) {
    const double w = right - left, h = top - bottom, p = far - near;
    vsg::dmat4 m; // identity
    m[0][0] = 2.0 / w;
    m[1][1] = 2.0 / h;
    m[2][2] = -2.0 / p;
    m[3][0] = -(right + left) / w;
    m[3][1] = -(top + bottom) / h;
    m[3][2] = -(far + near) / p;
    return m;
}

vsg::dmat4 translate(const vsg::dvec3& t) { return vsg::translate(t); }

// takram's getCubeSphereUv (clouds.glsl), for the weather's drift with the wind
vsg::dvec2 cubeSphereUv(const vsg::dvec3& position) {
    const vsg::dvec3 n = vsg::normalize(position);
    const vsg::dvec3 f(std::abs(n.x), std::abs(n.y), std::abs(n.z));
    const vsg::dvec3 c = n / std::max(f.x, std::max(f.y, f.z));
    vsg::dvec2 m;
    if (f.y > f.x && f.y > f.z) m = c.y > 0.0 ? vsg::dvec2(-n.x, n.z) : vsg::dvec2(n.x, n.z);
    else if (f.x > f.y && f.x > f.z) m = c.x > 0.0 ? vsg::dvec2(n.y, n.z) : vsg::dvec2(-n.y, n.z);
    else m = c.z > 0.0 ? vsg::dvec2(n.x, n.y) : vsg::dvec2(n.x, -n.y);
    const vsg::dvec2 m2(m.x * m.x, m.y * m.y);
    const double q = -2.0 * m2.x + 2.0 * m2.y - 3.0;
    vsg::dvec2 uv;
    uv.x = std::sqrt(std::max(1.5 + m2.x - m2.y - 0.5 * std::sqrt(std::max(-24.0 * m2.x + q * q, 0.0)), 0.0)) * (m.x > 0.0 ? 1.0 : -1.0);
    uv.y = std::sqrt(6.0 / (3.0 - uv.x * uv.x)) * m.y;
    return uv * 0.5 + vsg::dvec2(0.5, 0.5);
}

vsg::mat4 toFloat(const vsg::dmat4& m) { return vsg::mat4(m); }
vsg::vec4 v4(double x, double y, double z, double w) {
    return vsg::vec4(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(w));
}

} // namespace

AtmosphereTables loadAtmosphereTables(const std::filesystem::path& cloudsDir) {
    AtmosphereTables t;
    if (cloudsDir.empty()) return t;
    auto clamp = sampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, 0.0f);
    const auto info = [&](vsg::ref_ptr<vsg::Data> data) {
        return data ? vsg::ImageInfo::create(clamp, data, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) : vsg::ref_ptr<vsg::ImageInfo>{};
    };
    t.transmittance = info(halfTable(cloudsDir / "atmosphere" / "transmittance.bin", 256, 64, 1));
    t.irradiance = info(halfTable(cloudsDir / "atmosphere" / "irradiance.bin", 64, 16, 1));
    t.scattering = info(halfTable(cloudsDir / "atmosphere" / "scattering.bin", 256, 128, 32));
    if (!t.valid()) {
        LOG_WARN("world") << "the atmosphere's tables are not all in " << (cloudsDir / "atmosphere").string();
        return AtmosphereTables{};
    }
    return t;
}

CloudNoise loadCloudNoise(const std::filesystem::path& cloudsDir) {
    CloudNoise n;
    if (cloudsDir.empty()) return n;
    auto repeatMips = sampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT, 16.0f);
    const auto info = [&](vsg::ref_ptr<vsg::Data> data) {
        return data ? vsg::ImageInfo::create(repeatMips, data, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) : vsg::ref_ptr<vsg::ImageInfo>{};
    };
    n.shape = info(r8VolumeMips(cloudsDir / "shape.bin", 128));
    n.detail = info(r8VolumeMips(cloudsDir / "shape_detail.bin", 32));
    if (!n.valid()) {
        LOG_WARN("world") << "the cloud noises are not both in " << cloudsDir.string();
        return CloudNoise{};
    }
    return n;
}

vsg::ref_ptr<vsg::ImageInfo> loadBlueNoise(const std::filesystem::path& cloudsDir) {
    if (cloudsDir.empty()) return {};
    auto data = r8Volume(cloudsDir / "stbn.bin", 128, 128, 64);
    if (!data) return {};
    return vsg::ImageInfo::create(sampler(VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_REPEAT, 0.0f), data, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
}

vsg::ref_ptr<vsg::ShaderStage> loadCloudShader(const std::filesystem::path& shaders, const std::string& name, VkShaderStageFlagBits stage,
                                               const std::string& defines) {
    std::string src;
    if (!loadShaderSource(shaders, name, src)) {
        LOG_WARN("world") << "cannot read the shader " << (shaders / name).string();
        return {};
    }
    const auto at = src.find("#version 450\n");
    if (at == std::string::npos) return {};
    src.insert(at + 13, defines);
    return vsg::ShaderStage::create(stage, "main", src);
}

vsg::Descriptors AtmosphereTables::descriptors() const {
    const auto bind = [](vsg::ref_ptr<vsg::ImageInfo> info, std::uint32_t binding) {
        return vsg::ref_ptr<vsg::Descriptor>(vsg::DescriptorImage::create(info, binding, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER));
    };
    return {bind(transmittance, 6), bind(scattering, 7), bind(irradiance, 8), bind(scattering, 20)};
}

std::array<Clouds::Layer, 4> Clouds::layersFor(const Weather& w) {
    // CloudLayers.DEFAULT: r 750 m + 650, g 1000 m + 1200, b (cirrus) 7500 m + 500, a none; the low two moved onto
    // the weather's cloud base together, the cirrus kept above them
    const double base = std::max(w.baseM, 100.0), shift = base - 750.0;
    std::array<Layer, 4> l{};
    l[0] = {750.0 + shift, 650.0};
    l[1] = {1000.0 + shift, 1200.0};
    l[2] = {std::max(7500.0, l[1].altitudeM + l[1].heightM + 2000.0), 500.0};
    l[3] = {0.0, 0.0};
    return l;
}

double Clouds::coverageForCover(double cover) {
    if (cover <= 0.0) return 0.0;
    constexpr std::size_t n = std::size(kCurveCover);
    for (std::size_t i = 1; i < n; ++i)
        if (cover <= kCurveCover[i]) {
            const double t = (cover - kCurveCover[i - 1]) / (kCurveCover[i] - kCurveCover[i - 1]);
            return kCurveCoverage[i - 1] + t * (kCurveCoverage[i] - kCurveCoverage[i - 1]);
        }
    return kCurveCoverage[n - 1];
}

double Clouds::overcast(const Weather& w, double altitudeM) {
    const auto l = layersFor(w);
    const double base = l[0].altitudeM, top = l[1].altitudeM + l[1].heightM;
    // (0.7 at most: the eye adapts to a grey day; a camera exposing for sunshine would leave it near black)
    const double thick = 0.7 * smoothstep(0.6, 1.0, w.cover) * (1.0 + 0.2 * std::clamp(w.precipitation, 0.0, 1.0));
    // all of it below the base, none above the top, between in the cloud
    return std::min(thick * (1.0 - smoothstep(base, top, altitudeM)), 0.95);
}

Clouds::Clouds(vsg::ref_ptr<const vsg::Options> options, std::filesystem::path dir, vsg::ref_ptr<vsg::Data> earthMap,
               vsg::ref_ptr<vsg::Data> seaMap)
    : options_(std::move(options)), dir_(std::move(dir)), earthMap_(std::move(earthMap)), seaMap_(std::move(seaMap)),
      atmosphere_(loadAtmosphereTables(dir_)) {
    under_ = vsg::Group::create();
    over_ = vsg::Group::create();
    compute_ = vsg::Group::create();
    params_ = vsg::ubyteArray::create(sizeof(Params));
    std::memset(params_->dataPointer(), 0, sizeof(Params));
    params_->properties.dataVariance = vsg::DYNAMIC_DATA;
}

void Clouds::setScreen(std::uint32_t width, std::uint32_t height) {
    screen_ = vsg::vec2(static_cast<float>(std::max<std::uint32_t>(width, 1)), static_cast<float>(std::max<std::uint32_t>(height, 1)));
}

void Clouds::build(std::uint32_t width, std::uint32_t height, Compiler compiler) {
    if (built_ || dir_.empty()) return;
    const auto root = dir_ / "shaders";
    auto shadowShader = shaderFile(root, "shadow.comp", VK_SHADER_STAGE_COMPUTE_BIT);
    auto shadowResolveShader = shaderFile(root, "shadowResolve.comp", VK_SHADER_STAGE_COMPUTE_BIT);
    auto marchShader = shaderFile(root, "march.comp", VK_SHADER_STAGE_COMPUTE_BIT);
    auto resolveShader = shaderFile(root, "resolve.comp", VK_SHADER_STAGE_COMPUTE_BIT);
    auto vertex = vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", R"(#version 450
layout(push_constant) uniform PushConstants { mat4 projection; mat4 modelView; } pc;
out gl_PerVertex { vec4 gl_Position; };
void main() {
    vec2 p = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)");
    vsg::ref_ptr<vsg::ShaderStage> fragments[3];
    for (int m = 0; m < 3; ++m) fragments[m] = shaderFile(root, "composite.frag", VK_SHADER_STAGE_FRAGMENT_BIT, m);
    auto shaders = vsg::ShaderCompiler::create();
    bool ok = shaders->supported() && shadowShader && shadowResolveShader && marchShader && resolveShader;
    for (const auto& s : {shadowShader, shadowResolveShader, marchShader, resolveShader}) {
        if (!ok) break;
        vsg::ShaderStages one{s};
        ok = shaders->compile(one);
    }
    for (const auto& f : fragments) {
        if (!ok || !f) {
            ok = false;
            break;
        }
        vsg::ShaderStages two{vertex, f};
        ok = shaders->compile(two);
    }
    if (!ok) {
        LOG_WARN("world") << "cloud shaders did not build; no clouds";
        return;
    }

    // takram's textures and the atmosphere's tables
    auto stbn = r8Volume(dir_ / "stbn.bin", 128, 128, 64);
    auto shape = r8VolumeMips(dir_ / "shape.bin", 128);
    auto detail = r8VolumeMips(dir_ / "shape_detail.bin", 32);
    auto weatherMap = rgbaPng(dir_ / "local_weather.png");
    auto turbulence = rgbaPng(dir_ / "turbulence.png");
    if (!stbn || !shape || !detail || !weatherMap || !turbulence || !atmosphere_.valid()) {
        LOG_WARN("world") << "clouds: takram's textures are not all in " << dir_.string() << "; no clouds";
        return;
    }
    auto global = earthMap_;
    if (!global) {
        auto none = vsg::ubyteArray2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R8_UNORM});
        none->at(0, 0) = 0;
        global = none;
    }
    const auto texture = [](vsg::ref_ptr<vsg::Data> data, vsg::ref_ptr<vsg::Sampler> s, std::uint32_t binding) {
        return Bound{binding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, vsg::DescriptorImage::create(s, data, binding, 0)};
    };
    const auto repeatLinear = sampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT, 0.0f);
    const auto repeatMips = sampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT, 16.0f);
    const auto clampLinear = sampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, 0.0f);
    auto globalSampler = sampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT, 16.0f);
    globalSampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    const auto tables = atmosphere_.descriptors();
    const auto table = [&](std::size_t k, std::uint32_t binding) { return Bound{binding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, tables[k]}; };
    const std::vector<Bound> textures{
        texture(stbn, sampler(VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_REPEAT, 0.0f), 1),
        texture(weatherMap, repeatMips, 2),
        texture(shape, repeatMips, 3),
        texture(detail, repeatMips, 4),
        texture(turbulence, repeatMips, 5),
        table(0, 6),
        table(1, 7),
        table(2, 8),
        texture(global, globalSampler, 11),
        table(3, 20),
    };

    // the images: the shadow maps (this frame's march, its depth and velocity, and two resolved: this frame's and
    // the last), the march's (quarter size), and two resolved clouds (full size) - sized for the screen, not the
    // window as it starts, so a window maximised later uses a part of them
    unsigned sw = 0, sh = 0;
    platform::screenSize(sw, sh);
    const std::uint32_t fw = std::max(width, sw), fh = std::max(height, sh);
    fullAlloc_ = vsg::uivec2(std::max<std::uint32_t>(fw, 1), std::max<std::uint32_t>(fh, 1));
    lowAlloc_ = vsg::uivec2((fullAlloc_.x + 3) / 4, (fullAlloc_.y + 3) / 4);
    auto shadowCurrent = makeImage(VK_FORMAT_R32G32B32A32_SFLOAT, kShadowMap, kShadowMap, kCascades);
    auto shadowVelocity = makeImage(VK_FORMAT_R32G32B32A32_SFLOAT, kShadowMap, kShadowMap, kCascades);
    const vsg::ref_ptr<vsg::Image> shadowResolved[2] = {makeImage(VK_FORMAT_R32G32B32A32_SFLOAT, kShadowMap, kShadowMap, kCascades),
                                                        makeImage(VK_FORMAT_R32G32B32A32_SFLOAT, kShadowMap, kShadowMap, kCascades)};
    auto color = makeImage(VK_FORMAT_R16G16B16A16_SFLOAT, lowAlloc_.x, lowAlloc_.y, 1);
    auto depthVelocity = makeImage(VK_FORMAT_R32G32B32A32_SFLOAT, lowAlloc_.x, lowAlloc_.y, 1);
    const vsg::ref_ptr<vsg::Image> resolved[2] = {makeImage(VK_FORMAT_R16G16B16A16_SFLOAT, fullAlloc_.x, fullAlloc_.y, 1),
                                                  makeImage(VK_FORMAT_R16G16B16A16_SFLOAT, fullAlloc_.x, fullAlloc_.y, 1)};
    const auto storage = [](vsg::ref_ptr<vsg::Image> image, std::uint32_t binding) {
        return Bound{binding, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                     vsg::DescriptorImage::create(vsg::ImageInfo::create(vsg::ref_ptr<vsg::Sampler>{}, viewOf(image), VK_IMAGE_LAYOUT_GENERAL), binding, 0,
                                                  VK_DESCRIPTOR_TYPE_STORAGE_IMAGE)};
    };
    const auto sampled = [](vsg::ref_ptr<vsg::Image> image, vsg::ref_ptr<vsg::Sampler> s, std::uint32_t binding) {
        return Bound{binding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                     vsg::DescriptorImage::create(vsg::ImageInfo::create(s, viewOf(image), VK_IMAGE_LAYOUT_GENERAL), binding, 0,
                                                  VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER)};
    };
    auto nearest = sampler(VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, 0.0f);
    const Bound params{0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, vsg::DescriptorBuffer::create(params_, 0, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER)};

    // a pipeline layout and a set from what is bound
    const auto bindSet = [](VkPipelineBindPoint point, VkShaderStageFlags stages, const std::vector<Bound>& bound, bool push,
                            vsg::ref_ptr<vsg::PipelineLayout>& layout) {
        vsg::DescriptorSetLayoutBindings bindings;
        vsg::Descriptors descriptors;
        for (const auto& b : bound) {
            bindings.push_back({b.binding, b.type, 1, stages, nullptr});
            descriptors.push_back(b.descriptor);
        }
        layout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{vsg::DescriptorSetLayout::create(bindings)},
                                             push ? vsg::PushConstantRanges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}} : vsg::PushConstantRanges{});
        return vsg::BindDescriptorSet::create(point, layout, 0, vsg::DescriptorSet::create(layout->setLayouts[0], descriptors));
    };
    const VkShaderStageFlags cs = VK_SHADER_STAGE_COMPUTE_BIT;
    const auto with = [](std::vector<Bound> a, std::initializer_list<Bound> b) {
        a.insert(a.end(), b.begin(), b.end());
        return a;
    };
    const auto dispatch = [](vsg::ref_ptr<vsg::ShaderStage> s, vsg::ref_ptr<vsg::PipelineLayout> layout, vsg::ref_ptr<vsg::BindDescriptorSet> set,
                             std::uint32_t x, std::uint32_t y, std::uint32_t z, vsg::ref_ptr<vsg::Dispatch>& d) {
        auto g = vsg::StateGroup::create();
        g->add(vsg::BindComputePipeline::create(vsg::ComputePipeline::create(layout, s)));
        g->add(set);
        d = vsg::Dispatch::create(x, y, z);
        g->addChild(d);
        return g;
    };

    // the first frame: every image into the layout it stays in
    auto first = vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0);
    for (const auto& image : {shadowCurrent, shadowVelocity, shadowResolved[0], shadowResolved[1], color, depthVelocity, resolved[0], resolved[1]})
        first->imageMemoryBarriers.push_back(vsg::ImageMemoryBarrier::create(0, VK_ACCESS_SHADER_WRITE_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                                                                             VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, image,
                                                                             VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, image->arrayLayers}));
    firstSwitch_ = vsg::Switch::create();
    firstSwitch_->addChild(true, first);

    auto passes = vsg::Group::create();
    passes->addChild(firstSwitch_);
    {
        // the last frame's draws done with
        auto begin = vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                                  VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0);
        begin->memoryBarriers.push_back(vsg::MemoryBarrier::create(VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT));
        passes->addChild(begin);
    }
    const std::uint32_t shadowGroups = (kShadowMap + 7) / 8;
    vsg::ref_ptr<vsg::Dispatch> d;
    // the shadow maps
    {
        vsg::ref_ptr<vsg::PipelineLayout> layout;
        auto set = bindSet(VK_PIPELINE_BIND_POINT_COMPUTE, cs,
                           with({params, textures[0], textures[1], textures[2], textures[3], textures[4], textures[8]},
                                {storage(shadowCurrent, 12), storage(shadowVelocity, 13)}),
                           false, layout);
        passes->addChild(dispatch(shadowShader, layout, set, shadowGroups, shadowGroups, kCascades, d));
        passes->addChild(computeDone(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT));
    }
    shadowResolveSwitch_ = vsg::Switch::create();
    for (int k = 0; k < 2; ++k) {
        vsg::ref_ptr<vsg::PipelineLayout> layout;
        auto set = bindSet(VK_PIPELINE_BIND_POINT_COMPUTE, cs,
                           {params, sampled(shadowCurrent, nearest, 15), sampled(shadowVelocity, nearest, 16),
                            sampled(shadowResolved[1 - k], clampLinear, 17), storage(shadowResolved[k], 18)},
                           false, layout);
        shadowResolveSwitch_->addChild(k == 0, dispatch(shadowResolveShader, layout, set, shadowGroups, shadowGroups, kCascades, d));
    }
    passes->addChild(shadowResolveSwitch_);
    passes->addChild(computeDone(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT));
    // the march, reading this frame's shadow
    marchSwitch_ = vsg::Switch::create();
    for (int k = 0; k < 2; ++k) {
        vsg::ref_ptr<vsg::PipelineLayout> layout;
        auto bound = textures;
        bound.insert(bound.begin(), params);
        bound.push_back(sampled(shadowResolved[k], clampLinear, 9));
        bound.push_back(storage(color, 12));
        bound.push_back(storage(depthVelocity, 13));
        auto set = bindSet(VK_PIPELINE_BIND_POINT_COMPUTE, cs, bound, false, layout);
        marchSwitch_->addChild(k == 0, dispatch(marchShader, layout, set, (lowAlloc_.x + 7) / 8, (lowAlloc_.y + 7) / 8, 1, d));
        lowDispatches_.push_back(d);
    }
    passes->addChild(marchSwitch_);
    passes->addChild(computeDone(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT));
    // the resolve, into this frame's half of the pair from the other
    resolveSwitch_ = vsg::Switch::create();
    for (int k = 0; k < 2; ++k) {
        vsg::ref_ptr<vsg::PipelineLayout> layout;
        auto set = bindSet(VK_PIPELINE_BIND_POINT_COMPUTE, cs,
                           {params, sampled(color, clampLinear, 15), sampled(depthVelocity, nearest, 16), sampled(resolved[1 - k], clampLinear, 17),
                            storage(resolved[k], 18)},
                           false, layout);
        resolveSwitch_->addChild(k == 0, dispatch(resolveShader, layout, set, (fullAlloc_.x + 7) / 8, (fullAlloc_.y + 7) / 8, 1, d));
        fullDispatches_.push_back(d);
    }
    passes->addChild(resolveSwitch_);
    passes->addChild(computeDone(VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT));
    computeSwitch_ = vsg::Switch::create();
    computeSwitch_->addChild(true, passes);

    // the draws
    const VkShaderStageFlags fs = VK_SHADER_STAGE_FRAGMENT_BIT;
    const auto draw = [&](int mode, int k) {
        vsg::ref_ptr<vsg::PipelineLayout> layout;
        auto set = bindSet(VK_PIPELINE_BIND_POINT_GRAPHICS, fs,
                           {params, sampled(shadowResolved[k], clampLinear, 9), sampled(resolved[k], nearest, 15),
                            sampled(depthVelocity, nearest, 16)},
                           true, layout);
        auto raster = vsg::RasterizationState::create();
        raster->cullMode = VK_CULL_MODE_NONE;
        auto depth = vsg::DepthStencilState::create();
        depth->depthWriteEnable = mode == 2 ? VK_TRUE : VK_FALSE;
        depth->depthTestEnable = mode == 0 ? VK_FALSE : VK_TRUE;
        depth->depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;
        auto blend = vsg::ColorBlendState::create();
        const VkColorComponentFlags rgba = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        if (mode == 0) // multiplied in
            blend->attachments = {VkPipelineColorBlendAttachmentState{VK_TRUE, VK_BLEND_FACTOR_ZERO, VK_BLEND_FACTOR_SRC_COLOR, VK_BLEND_OP_ADD,
                                                                      VK_BLEND_FACTOR_ZERO, VK_BLEND_FACTOR_ONE, VK_BLEND_OP_ADD, rgba}};
        else if (mode == 2) // depth alone
            blend->attachments = {VkPipelineColorBlendAttachmentState{VK_FALSE, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ZERO, VK_BLEND_OP_ADD,
                                                                      VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ZERO, VK_BLEND_OP_ADD, 0}};
        else // premultiplied over
            blend->attachments = {VkPipelineColorBlendAttachmentState{VK_TRUE, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
                                                                      VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD, rgba}};
        vsg::GraphicsPipelineStates states{vsg::VertexInputState::create(), vsg::InputAssemblyState::create(), raster, vsg::MultisampleState::create(),
                                           blend, depth};
        auto g = vsg::StateGroup::create();
        g->add(vsg::BindGraphicsPipeline::create(vsg::GraphicsPipeline::create(layout, vsg::ShaderStages{vertex, fragments[mode]}, states)));
        g->add(set);
        g->addChild(vsg::Draw::create(3, 1, 0, 0));
        return g;
    };
    underSwitch_ = vsg::Switch::create(); // (the ground's shade, from this frame's shadow map: one half of the pair or the other)
    for (int k = 0; k < 2; ++k) underSwitch_->addChild(k == 0, draw(0, k));
    overSwitch_ = vsg::Switch::create(); // (the cloud and its depth, from one half of the pair and from the other)
    for (int k = 0; k < 2; ++k)
        for (int mode = 1; mode < 3; ++mode) overSwitch_->addChild(k == 0, draw(mode, k));

    // the sea: its passes (each frame: the spectrum carried to the moment, the inverse FFT, the slopes and foam
    // unpacked and their levels made; the spectrum itself only when the wind changes) and its draws (one with each
    // half of the clouds' shadow pair)
    if (seaMap_) [&] {
        const char* names[] = {"ocean/spectrum.comp", "ocean/modulate.comp", "ocean/fft.comp", "ocean/transpose.comp", "ocean/unpack.comp",
                               "ocean/downsample.comp"};
        vsg::ref_ptr<vsg::ShaderStage> stages[6];
        for (int i = 0; i < 6; ++i) {
            stages[i] = shaderFile(root, names[i], VK_SHADER_STAGE_COMPUTE_BIT);
            vsg::ShaderStages one{stages[i]};
            if (!stages[i] || !shaders->compile(one)) {
                LOG_WARN("world") << "sea: " << names[i] << " did not build; no sea";
                return;
            }
        }
        auto seaVertex = shaderFile(root, "ocean/ocean.vert", VK_SHADER_STAGE_VERTEX_BIT);
        auto seaFragment = shaderFile(root, "ocean/ocean.frag", VK_SHADER_STAGE_FRAGMENT_BIT);
        vsg::ShaderStages two{seaVertex, seaFragment};
        if (!seaVertex || !seaFragment || !shaders->compile(two)) {
            LOG_WARN("world") << "sea: ocean/ocean.vert or ocean.frag did not build; no sea";
            return;
        }
        auto spectrum = makeImage(VK_FORMAT_R16G16B16A16_SFLOAT, kSeaMap, kSeaMap, kSeaCascades);
        auto slopes = makeImage(VK_FORMAT_R16G16B16A16_SFLOAT, kSeaMap, kSeaMap, kSeaCascades);
        auto foam = makeImage(VK_FORMAT_R16G16B16A16_SFLOAT, kSeaMap, kSeaMap, kSeaCascades);
        auto displacement = makeImage(VK_FORMAT_R16G16B16A16_SFLOAT, kSeaMap, kSeaMap, kSeaCascades);
        slopes->mipLevels = foam->mipLevels = displacement->mipLevels = kSeaLevels;
        auto fft = vsg::vec2Array::create(kSeaCascades * 2 * kSeaSpectra * kSeaMap * kSeaMap);
        const Bound fftBuffer{25, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, vsg::DescriptorBuffer::create(fft, 25, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER)};
        const Bound butterflies{26, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                vsg::DescriptorBuffer::create(fftButterflies(kSeaMap), 26, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER)};
        const auto level = [](vsg::ref_ptr<vsg::Image> image, std::uint32_t l, std::uint32_t binding) {
            return Bound{binding, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                         vsg::DescriptorImage::create(vsg::ImageInfo::create(vsg::ref_ptr<vsg::Sampler>{}, levelOf(image, l), VK_IMAGE_LAYOUT_GENERAL),
                                                      binding, 0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE)};
        };
        const auto step = [] {
            auto b = vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0);
            b->memoryBarriers.push_back(vsg::MemoryBarrier::create(VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT));
            return b;
        };

        auto seaFirst = vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0);
        for (const auto& image : {spectrum, slopes, foam, displacement})
            seaFirst->imageMemoryBarriers.push_back(vsg::ImageMemoryBarrier::create(
                0, VK_ACCESS_SHADER_WRITE_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL, VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, image,
                VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, image->mipLevels, 0, image->arrayLayers}));
        seaFirstSwitch_ = vsg::Switch::create();
        seaFirstSwitch_->addChild(true, seaFirst);

        auto sea = vsg::Group::create();
        sea->addChild(seaFirstSwitch_);
        {
            // the last frame's draw done with the maps
            auto begin = vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0);
            begin->memoryBarriers.push_back(vsg::MemoryBarrier::create(VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_SHADER_WRITE_BIT));
            sea->addChild(begin);
        }
        const std::uint32_t tiles = kSeaMap / 16, spectra = kSeaCascades * kSeaSpectra;
        vsg::ref_ptr<vsg::PipelineLayout> layout;
        seaSpectrumSwitch_ = vsg::Switch::create();
        {
            auto group = vsg::Group::create();
            auto set = bindSet(VK_PIPELINE_BIND_POINT_COMPUTE, cs, {params, storage(spectrum, 24)}, false, layout);
            group->addChild(dispatch(stages[0], layout, set, tiles, tiles, kSeaCascades, d));
            group->addChild(step());
            seaSpectrumSwitch_->addChild(true, group);
        }
        sea->addChild(seaSpectrumSwitch_);
        {
            auto set = bindSet(VK_PIPELINE_BIND_POINT_COMPUTE, cs, {params, storage(spectrum, 24), fftBuffer}, false, layout);
            sea->addChild(dispatch(stages[1], layout, set, tiles, tiles, kSeaCascades, d));
            sea->addChild(step());
        }
        {
            vsg::ref_ptr<vsg::PipelineLayout> fftLayout, transposeLayout;
            auto fftSet = bindSet(VK_PIPELINE_BIND_POINT_COMPUTE, cs, {params, fftBuffer, butterflies}, false, fftLayout);
            auto transposeSet = bindSet(VK_PIPELINE_BIND_POINT_COMPUTE, cs, {params, fftBuffer}, false, transposeLayout);
            sea->addChild(dispatch(stages[2], fftLayout, fftSet, 1, kSeaMap, spectra, d));
            sea->addChild(step());
            sea->addChild(dispatch(stages[3], transposeLayout, transposeSet, kSeaMap / 32, kSeaMap / 32, spectra, d));
            sea->addChild(step());
            sea->addChild(dispatch(stages[2], fftLayout, fftSet, 1, kSeaMap, spectra, d));
            sea->addChild(step());
        }
        {
            auto set = bindSet(VK_PIPELINE_BIND_POINT_COMPUTE, cs, {params, fftBuffer, level(slopes, 0, 27), level(foam, 0, 28), level(displacement, 0, 29)},
                               false, layout);
            sea->addChild(dispatch(stages[4], layout, set, tiles, tiles, kSeaCascades, d));
            sea->addChild(step());
        }
        for (std::uint32_t l = 1; l < kSeaLevels; ++l) {
            const std::uint32_t size = kSeaMap >> l, groups = (size + 7) / 8;
            auto set = bindSet(VK_PIPELINE_BIND_POINT_COMPUTE, cs,
                               {level(slopes, l - 1, 1), level(slopes, l, 2), level(foam, l - 1, 3), level(foam, l, 4), level(displacement, l - 1, 5),
                                level(displacement, l, 6)},
                               false, layout);
            sea->addChild(dispatch(stages[5], layout, set, groups, groups, kSeaCascades, d));
            sea->addChild(l + 1 < kSeaLevels ? step() : computeDone(VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT));
        }
        seaComputeSwitch_ = vsg::Switch::create();
        seaComputeSwitch_->addChild(true, sea);

        // the draw: ocean.vert's grid over the frame (a vertex every 4 pixels of the largest screen, and a margin round
        // it the waves' displacement may pull in), blended as out = sea + frame * its alpha (the imagery seen through
        // it), depth-tested below its surface (ocean.frag) and writing that depth, so a wave's crest hides the sea
        // behind it
        const std::uint32_t cols = std::max<std::uint32_t>(fullAlloc_.x / 8, 16), rows = std::max<std::uint32_t>(fullAlloc_.y / 8, 16);
        auto gridUv = vsg::vec2Array::create((cols + 1) * (rows + 1));
        for (std::uint32_t j = 0; j <= rows; ++j)
            for (std::uint32_t i = 0; i <= cols; ++i)
                gridUv->at(j * (cols + 1) + i) = vsg::vec2(-kSeaGridMargin + (1.0f + 2.0f * kSeaGridMargin) * static_cast<float>(i) / static_cast<float>(cols),
                                                           -kSeaGridMargin + (1.0f + 2.0f * kSeaGridMargin) * static_cast<float>(j) / static_cast<float>(rows));
        auto gridIndices = vsg::uintArray::create(cols * rows * 6);
        for (std::uint32_t j = 0, n = 0; j < rows; ++j)
            for (std::uint32_t i = 0; i < cols; ++i) {
                const std::uint32_t v = j * (cols + 1) + i;
                for (const std::uint32_t k : {v, v + 1, v + cols + 1, v + 1, v + cols + 2, v + cols + 1}) gridIndices->at(n++) = k;
            }
        seaGridCell_ = (1.0f + 2.0f * kSeaGridMargin) / static_cast<float>(cols);
        shoreData_ = vsg::ubyteArray2D::create(kShoreSize, kShoreSize, vsg::Data::Properties{VK_FORMAT_R8_UNORM});
        std::fill(shoreData_->begin(), shoreData_->end(), std::uint8_t{255}); // (far out to sea)
        shoreData_->properties.dataVariance = vsg::DYNAMIC_DATA;
        auto mapSampler = sampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT, 16.0f);
        mapSampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        auto waveSampler = sampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT, static_cast<float>(kSeaLevels - 1));
        seaDrawSwitch_ = vsg::Switch::create();
        for (int k = 0; k < 2; ++k) {
            vsg::ref_ptr<vsg::PipelineLayout> drawLayout;
            auto set = bindSet(VK_PIPELINE_BIND_POINT_GRAPHICS, VK_SHADER_STAGE_VERTEX_BIT | fs,
                               {params, table(0, 6), table(1, 7), table(2, 8), table(3, 20), sampled(shadowResolved[k], clampLinear, 9),
                                sampled(slopes, waveSampler, 21), sampled(foam, waveSampler, 22), texture(seaMap_, mapSampler, 23),
                                sampled(displacement, waveSampler, 30), sampled(resolved[k], clampLinear, 31),
                                Bound{32, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, vsg::DescriptorImage::create(clampLinear, shoreData_, 32, 0)}},
                               true, drawLayout);
            auto raster = vsg::RasterizationState::create();
            raster->cullMode = VK_CULL_MODE_NONE;
            auto depth = vsg::DepthStencilState::create();
            depth->depthWriteEnable = VK_TRUE;
            depth->depthTestEnable = VK_TRUE;
            depth->depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;
            auto blend = vsg::ColorBlendState::create();
            const VkColorComponentFlags rgb = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT;
            blend->attachments = {VkPipelineColorBlendAttachmentState{VK_TRUE, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_SRC_ALPHA, VK_BLEND_OP_ADD,
                                                                      VK_BLEND_FACTOR_ZERO, VK_BLEND_FACTOR_ONE, VK_BLEND_OP_ADD, rgb}};
            auto input = vsg::VertexInputState::create();
            input->vertexBindingDescriptions = {VkVertexInputBindingDescription{0, sizeof(vsg::vec2), VK_VERTEX_INPUT_RATE_VERTEX}};
            input->vertexAttributeDescriptions = {VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32_SFLOAT, 0}};
            vsg::GraphicsPipelineStates states{input, vsg::InputAssemblyState::create(), raster, vsg::MultisampleState::create(), blend, depth};
            auto g = vsg::StateGroup::create();
            g->add(vsg::BindGraphicsPipeline::create(vsg::GraphicsPipeline::create(drawLayout, vsg::ShaderStages{seaVertex, seaFragment}, states)));
            g->add(set);
            auto grid = vsg::VertexIndexDraw::create();
            grid->assignArrays(vsg::DataList{gridUv});
            grid->assignIndices(gridIndices);
            grid->indexCount = static_cast<std::uint32_t>(gridIndices->size());
            grid->instanceCount = 1;
            g->addChild(grid);
            seaDrawSwitch_->addChild(k == 0, g);
        }
        seaBuilt_ = true;
    }();

    bool compiled = true;
    for (auto node : std::initializer_list<vsg::ref_ptr<vsg::Node>>{computeSwitch_, underSwitch_, overSwitch_})
        compiled = (!compiler || compiler(node)) && compiled;
    if (!compiled) {
        LOG_WARN("world") << "could not compile the clouds";
        return;
    }
    if (seaBuilt_ && compiler && !(compiler(seaComputeSwitch_) && compiler(seaDrawSwitch_))) {
        LOG_WARN("world") << "could not compile the sea";
        seaBuilt_ = false;
    }
    compute_->addChild(computeSwitch_);
    under_->addChild(underSwitch_);
    over_->addChild(overSwitch_);
    if (seaBuilt_) {
        compute_->addChild(seaComputeSwitch_);
        under_->addChild(seaDrawSwitch_);
    }
    setScreen(width, height);
    built_ = true;
}

void Clouds::update(const vsg::dmat4& view, const vsg::dmat4& projection, const vsg::dvec3& eyeEcef, const vsg::EllipsoidModel& ellipsoid,
                    const vsg::dvec3& toSunEcef, double groundM, double timeS) {
    if (!built_) return;
    seaShown_ = seaBuilt_ && seaVisible_;
    ++frame_;
    const bool wasShown = shown_;
    const vsg::dvec3 lla = ellipsoid.convertECEFToLatLongAltitude(eyeEcef);
    // the global map's mean cover round the eye (fsim/coverage.glsl lays the world's cover out as the map's clouds lie)
    if (earthMap_ && (std::abs(lla.x - mapAroundAt_.x) * 111000.0 > kAroundMoved ||
                      std::abs(lla.y - mapAroundAt_.y) * 111000.0 * std::cos(vsg::radians(lla.x)) > kAroundMoved)) {
        mapAround_ = mapCoverAround(*earthMap_, lla.x, lla.y, kAroundRadius);
        mapAroundAt_ = vsg::dvec2(lla.x, lla.y);
    }
    // no weather given yet - a viewer waiting for its world: the map's cover round the eye, so the clouds seen from
    // far off are there as the eye comes down to them (the world's none took them all away near the ground: they
    // vanished as the view zoomed in, until a world gave its weather)
    if (!weatherGiven_ && earthMap_) {
        weather_ = given_;
        weather_.cover = mapAround_;
    }
    // (the cover where the eye is; with the Earth's map there is cloud somewhere whatever it is)
    shown_ = visible_ && (weather_.cover > 0.005 || earthMap_);
    // the images are put in their layout on the first frame recorded with the clouds shown, and only then
    if (shown_ && ++shownFrames_ == 2) firstSwitch_->setAllChildren(false);
    computeSwitch_->setAllChildren(shown_);
    parity_ ^= 1;
    const auto pick = [&](vsg::Switch& s, std::size_t per) {
        for (std::size_t c = 0; c < s.children.size(); ++c) s.children[c].mask = shown_ && static_cast<int>(c / per) == parity_ ? vsg::MASK_ALL : vsg::MASK_OFF;
    };
    pick(*shadowResolveSwitch_, 1);
    pick(*marchSwitch_, 1);
    pick(*resolveSwitch_, 1);
    pick(*underSwitch_, 1);
    pick(*overSwitch_, 2);
    if (seaBuilt_) {
        seaComputeSwitch_->setAllChildren(seaShown_);
        // (its draw reads the clouds' shadow from the half of the pair this frame's resolve writes)
        for (std::size_t c = 0; c < seaDrawSwitch_->children.size(); ++c)
            seaDrawSwitch_->children[c].mask = seaShown_ && static_cast<int>(c) == parity_ ? vsg::MASK_ALL : vsg::MASK_OFF;
        if (seaShown_ && ++seaFrames_ == 2) seaFirstSwitch_->setAllChildren(false);
    }
    if (!shown_) {
        haveLast_ = false;
        if (!seaShown_) return;
    }

    Params p{};
    const vsg::dvec3 sun = vsg::normalize(toSunEcef);

    // the camera: "world" is ECEF less the eye
    const vsg::dmat4 viewWorld = view * translate(eyeEcef);
    const vsg::dmat4 inverseView = vsg::inverse(viewWorld);
    const vsg::dmat4 inverseProjection = vsg::inverse(projection);
    const auto unproject = [&](double depth) {
        const vsg::dvec4 h = inverseProjection * vsg::dvec4(0.0, 0.0, depth, 1.0);
        return std::abs(h.w) > 1e-30 ? -h.z / h.w : 1e30;
    };
    const double cameraNear = std::max(unproject(1.0), 1e-3), cameraFar = std::min(std::max(unproject(0.0), cameraNear * 2.0), 1e9);
    const vsg::dmat4 reprojection = (haveLast_ ? lastViewProjection_ : projection * view) * translate(eyeEcef);
    p.inverseProjection = toFloat(inverseProjection);
    p.projection = toFloat(projection);
    p.inverseView = toFloat(inverseView);
    p.view = toFloat(viewWorld);
    p.reprojection = toFloat(reprojection);
    p.viewReprojection = toFloat(reprojection * inverseView);

    // takram's altitude correction: the osculating sphere of their bottom radius under the eye
    const vsg::dvec3 surface = ellipsoid.convertLatLongAltitudeToECEF(vsg::dvec3(lla.x, lla.y, 0.0));
    const double a2 = ellipsoid.radiusEquator() * ellipsoid.radiusEquator(), b2 = ellipsoid.radiusPolar() * ellipsoid.radiusPolar();
    const vsg::dvec3 normal = vsg::normalize(vsg::dvec3(surface.x / a2, surface.y / a2, surface.z / b2));
    const vsg::dvec3 correction = -(surface - normal * kBottomRadius);

    // the layers on the weather, and the coverage where the eye is
    const auto layers = layersFor(weather_);
    const double coverage = coverageForCover(std::clamp(weather_.cover, 0.0, 1.0));
    {
        // CloudLayers.packIntervalHeights: the gaps between the layers, where the march strides
        struct Entry {
            double value;
            int flag;
        };
        std::array<Entry, 8> e{};
        for (int i = 0; i < 4; ++i) {
            e[static_cast<std::size_t>(i)] = {layers[static_cast<std::size_t>(i)].altitudeM, 0};
            e[static_cast<std::size_t>(i + 4)] = {layers[static_cast<std::size_t>(i)].altitudeM + layers[static_cast<std::size_t>(i)].heightM, 1};
        }
        std::sort(e.begin(), e.end(), [](const Entry& a, const Entry& b) { return a.value != b.value ? a.value < b.value : a.flag < b.flag; });
        double mins[3] = {0.0, 0.0, 0.0}, maxs[3] = {0.0, 0.0, 0.0};
        int interval = 0, balance = 0;
        for (std::size_t i = 0; i < e.size(); ++i) {
            if (balance == 0 && i > 0 && interval < 3) {
                mins[interval] = e[i - 1].value;
                maxs[interval] = e[i].value;
                ++interval;
            }
            balance += e[i].flag == 0 ? 1 : -1;
        }
        p.minIntervalHeights = v4(mins[0], mins[1], mins[2], 0.0);
        p.maxIntervalHeights = v4(maxs[0], maxs[1], maxs[2], 0.0);
    }
    p.minLayerHeights = v4(layers[0].altitudeM, layers[1].altitudeM, layers[2].altitudeM, layers[3].altitudeM);
    p.maxLayerHeights = v4(layers[0].altitudeM + layers[0].heightM, layers[1].altitudeM + layers[1].heightM, layers[2].altitudeM + layers[2].heightM,
                           layers[3].altitudeM + layers[3].heightM);
    p.densityScales = vsg::vec4(0.2f, 0.2f, 0.003f, 0.2f);
    p.shapeAmounts = vsg::vec4(1.0f, 1.0f, 0.4f, 1.0f);
    p.shapeDetailAmounts = vsg::vec4(1.0f, 1.0f, 0.0f, 1.0f);
    p.weatherExponents = vsg::vec4(1.0f, 1.0f, 1.0f, 1.0f);
    p.shapeAlteringBiases = vsg::vec4(0.35f, 0.35f, 0.35f, 0.35f);
    p.coverageFilterWidths = vsg::vec4(0.6f, 0.6f, 0.5f, 0.6f);
    p.expTerms = vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f);
    p.exponents = vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f);
    p.linearTerms = vsg::vec4(0.75f, 0.75f, 0.75f, 0.75f);
    p.constantTerms = vsg::vec4(0.25f, 0.25f, 0.25f, 0.25f);
    const double minHeight = layers[0].altitudeM, maxHeight = layers[2].altitudeM + layers[2].heightM;
    const double shadowBottom = layers[0].altitudeM, shadowTop = layers[1].altitudeM + layers[1].heightM;
    p.heights = v4(minHeight, maxHeight, shadowTop, shadowBottom);
    p.shadowLayerMask = vsg::vec4(1.0f, 1.0f, 0.0f, 0.0f);

    // the wind carries the weather and the shapes (their velocities, applied to the offsets as CloudsEffect does)
    const double dt = lastTime_ >= 0.0 ? std::clamp(timeS - lastTime_, 0.0, 1.0) : 0.0;
    lastTime_ = timeS;
    {
        const double lat = vsg::radians(lla.x), lon = vsg::radians(lla.y);
        const vsg::dvec3 E(-std::sin(lon), std::cos(lon), 0.0);
        const vsg::dvec3 N(-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat));
        const double from = vsg::radians(weather_.windFromDeg), speed = std::max(weather_.windMs, 0.0);
        const vsg::dvec3 wind = (E * -std::sin(from) + N * -std::cos(from)) * speed; // (towards)
        const vsg::dvec3 at = eyeEcef + correction;
        const vsg::dvec2 uv0 = cubeSphereUv(at), uvE = cubeSphereUv(at + E * 1000.0), uvN = cubeSphereUv(at + N * 1000.0);
        const double we = vsg::dot(wind, E), wn = vsg::dot(wind, N);
        if (vsg::length(uvE - uv0) < 0.01 && vsg::length(uvN - uv0) < 0.01) // (not across a face's edge)
            weatherOffset_ -= ((uvE - uv0) * we + (uvN - uv0) * wn) / 1000.0 * kWeatherRepeat * dt;
        shapeOffset_ -= wind * (kShapeRepeat * dt);
        detailOffset_ -= wind * (kDetailRepeat * dt);
        const auto wrap = [](vsg::dvec3& v) { // (whole periods of the textures: invisible)
            v.x = std::fmod(v.x, 1000.0);
            v.y = std::fmod(v.y, 1000.0);
            v.z = std::fmod(v.z, 1000.0);
        };
        wrap(shapeOffset_);
        wrap(detailOffset_);
    }
    p.weather = v4(kWeatherRepeat, kWeatherRepeat, weatherOffset_.x, weatherOffset_.y);
    p.shapeRepeat = v4(kShapeRepeat, kShapeRepeat, kShapeRepeat, 0.0);
    p.shapeOffset = v4(shapeOffset_.x, shapeOffset_.y, shapeOffset_.z, 0.0);
    p.shapeDetailRepeat = v4(kDetailRepeat, kDetailRepeat, kDetailRepeat, 0.0);
    p.shapeDetailOffset = v4(detailOffset_.x, detailOffset_.y, detailOffset_.z, 0.0);
    p.turbulence = v4(kTurbulenceRepeat, kTurbulenceRepeat, 350.0, 0.0);

    // CascadedShadowMaps.update (three-csm's, as takram has it): practical splits up to 100 km, a square orthographic
    // frustum round each part of the camera's, snapped to the map's texels, the sun far behind it
    const double shadowFar = std::min(kShadowMaxFar, cameraFar);
    double splits[kCascades];
    for (std::uint32_t i = 0; i < kCascades; ++i) {
        const double f = static_cast<double>(i + 1) / kCascades;
        const double uniform = (cameraNear + (shadowFar - cameraNear) * f) / shadowFar;
        const double logarithmic = cameraNear * std::pow(shadowFar / cameraNear, f) / shadowFar;
        splits[i] = uniform + (logarithmic - uniform) * kSplitLambda;
    }
    {
        // the camera's frustum to the shadows' far end (view space): FrustumCorners.setFromCamera
        vsg::dvec3 nearC[4], farC[4];
        const double cx[4] = {1.0, 1.0, -1.0, -1.0}, cy[4] = {1.0, -1.0, -1.0, 1.0};
        for (int i = 0; i < 4; ++i) {
            const vsg::dvec4 h = inverseProjection * vsg::dvec4(cx[i], cy[i], 1.0, 1.0);
            nearC[i] = vsg::dvec3(h.x, h.y, h.z) / h.w;
            farC[i] = nearC[i] * std::min(shadowFar / std::abs(nearC[i].z), 1e12);
        }
        const vsg::dvec3 up(0.0, 1.0, 0.0);
        const vsg::dmat4 lightOrientation = threeLookAt(vsg::dvec3(0.0, 0.0, 0.0), -sun, up);
        const vsg::dmat4 lightFromWorld = vsg::inverse(lightOrientation);
        const vsg::dmat4 cameraToLight = lightFromWorld * inverseView;
        const vsg::dvec4 eyeLight = lightFromWorld * vsg::dvec4(eyeEcef, 0.0); // (to snap on a grid fixed to the Earth)
        const vsg::dvec3 eyeInLight(eyeLight.x, eyeLight.y, eyeLight.z);
        const double zenith = vsg::dot(sun, normal);
        const double distance = 1e6 + (1e3 - 1e6) * zenith;
        for (std::uint32_t c = 0; c < kCascades; ++c) {
            vsg::dvec3 n[4], f[4];
            for (int i = 0; i < 4; ++i) {
                n[i] = c == 0 ? nearC[i] : nearC[i] + (farC[i] - nearC[i]) * splits[c - 1];
                f[i] = c == kCascades - 1 ? farC[i] : nearC[i] + (farC[i] - nearC[i]) * splits[c];
            }
            double diagonal = std::max(vsg::length(f[0] - f[2]), vsg::length(f[0] - n[2]));
            const double depth = f[0].z / (shadowFar - cameraNear); // (fade: the bounds grown by its width)
            diagonal += 0.25 * depth * depth * (shadowFar - cameraNear);
            const double radius = diagonal * 0.5;
            const vsg::dmat4 proj = threeOrthographic(-radius, radius, radius, -radius, 0.0, radius * 2.0);
            vsg::dvec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
            for (int i = 0; i < 4; ++i)
                for (const auto& q : {n[i], f[i]}) {
                    const vsg::dvec4 l = cameraToLight * vsg::dvec4(q, 1.0);
                    lo = vsg::dvec3(std::min(lo.x, l.x), std::min(lo.y, l.y), std::min(lo.z, l.z));
                    hi = vsg::dvec3(std::max(hi.x, l.x), std::max(hi.y, l.y), std::max(hi.z, l.z));
                }
            vsg::dvec3 center = (lo + hi) * 0.5;
            center.z = hi.z;
            const double texel = 2.0 * radius / kShadowMap;
            center.x = std::round((center.x + eyeInLight.x) / texel) * texel - eyeInLight.x;
            center.y = std::round((center.y + eyeInLight.y) / texel) * texel - eyeInLight.y;
            const vsg::dvec4 centerWorld = lightOrientation * vsg::dvec4(center, 0.0);
            center = vsg::dvec3(centerWorld.x, centerWorld.y, centerWorld.z);
            const vsg::dvec3 position = center + sun * distance;
            vsg::dmat4 inverseLightView = threeLookAt(center, position, up);
            inverseLightView[3] = vsg::dvec4(position, 1.0);
            const vsg::dmat4 matrix = proj * vsg::inverse(inverseLightView);
            p.shadow[c] = toFloat(matrix);
            p.inverseShadow[c] = toFloat(vsg::inverse(matrix));
            p.shadowReprojection[c] = toFloat((haveLast_ ? lastShadow_[c] : matrix * translate(-eyeEcef)) * translate(eyeEcef));
            lastShadow_[c] = matrix * translate(-eyeEcef);
            p.shadowIntervals[c] = v4(c == 0 ? 0.0 : splits[c - 1], splits[c], texel, 0.0); // (z: a texel's width)
        }
    }

    // the march's sizes: the window's quarter of the images; the frame's Bayer offset (bayer.ts)
    const vsg::uivec2 window(static_cast<std::uint32_t>(screen_.x), static_cast<std::uint32_t>(screen_.y));
    const vsg::uivec2 low(std::min(lowAlloc_.x, (window.x + 3) / 4), std::min(lowAlloc_.y, (window.y + 3) / 4));
    for (auto& dispatch : lowDispatches_) {
        dispatch->groupCountX = (low.x + 7) / 8;
        dispatch->groupCountY = (low.y + 7) / 8;
    }
    for (auto& dispatch : fullDispatches_) {
        dispatch->groupCountX = (std::min(fullAlloc_.x, window.x) + 7) / 8;
        dispatch->groupCountY = (std::min(fullAlloc_.y, window.y) + 7) / 8;
    }
    const bool resized = low.x != lastLow_.x || low.y != lastLow_.y;
    lastLow_ = low;
    static constexpr int kBayer[16] = {0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5};
    const int f16 = static_cast<int>(frame_ % 16);
    int cell = 0;
    while (kBayer[cell] != f16) ++cell;
    p.screen = v4(std::min(fullAlloc_.x, window.x), std::min(fullAlloc_.y, window.y), low.x, low.y);
    p.alloc = v4(lowAlloc_.x, lowAlloc_.y, fullAlloc_.x, fullAlloc_.y);
    p.frame = v4((cell % 4 + 0.5) / 4.0, (cell / 4 + 0.5) / 4.0, static_cast<double>(frame_ % 4096), 0.25);
    p.nearFar = v4(cameraNear, cameraFar, shadowFar, 6.0);
    p.shadowTexel = v4(1.0 / kShadowMap, 1.0 / kShadowMap, kShadowMap, kShadowMap);

    p.camera = v4(eyeEcef.x, eyeEcef.y, eyeEcef.z, lla.z);
    p.altitudeCorrection = v4(correction.x, correction.y, correction.z, kBottomRadius);
    p.sun = v4(sun.x, sun.y, sun.z, coverage);
    // the march: the "high" preset
    p.march = v4(1e-5, 1e-5, 1e-2, 2e5);
    p.march2 = v4(50.0, 1000.0, 1.01, 500.0);
    p.secondary = v4(2.0, 3.0, 100.0, 2.0);
    p.light = v4(1.0, 1.0, 0.8, 150.0);
    // the history: drawn on when there is one (shown last frame, the same size)
    cloudsHistory_ = wasShown && haveLast_ && !resized;
    shadowHistory_ = wasShown && haveLast_;
    p.medium = v4(1.0, 0.0, cloudsHistory_ ? 1.0 : 0.0, shadowHistory_ ? 1.0 : 0.0);
    p.shadowMarch = v4(50.0, 100.0, 1000.0, 2.0);
    p.shadowMarch2 = v4(1e-5, 1e-5, 1e-4, 0.0);
    // the ground: its height, and the share of its light that is the sun's (an overcast's shade is everywhere, and
    // world::setDaylight has it)
    const Daylight d = daylightAt(toSunEcef, normal);
    const double share = d.sun / std::max(d.sun + d.sky + d.ambient, 1e-3f) * (1.0 - smoothstep(0.6, 1.0, weather_.cover));
    p.ground = v4(groundM, share, 0.0, 0.0);
    // the viewer's night light (its ambient, as the day fades: their atmosphere has no moon or night sky, and the
    // clouds were black over a moonlit ground)
    const double sunUp = vsg::dot(sun, vsg::normalize(eyeEcef));
    const double night = (1.0 - std::clamp((sunUp + 0.10) / 0.25, 0.0, 1.0)) * d.ambient;
    p.night = v4(d.ambientColour.x * night, d.ambientColour.y * night, d.ambientColour.z * night, 0.0);
    p.global = v4(earthMap_ ? 1.0 : 0.0, kGlobalFrom, kGlobalTo, smoothstep(kGlobalHighFrom, kGlobalHighTo, lla.z));
    // the world's cover, and the map's mean cover round the eye: near the eye the world's cover is laid out as the
    // map's clouds lie there (fsim/coverage.glsl)
    p.ground.z = static_cast<float>(std::clamp(weather_.cover, 0.0, 1.0));
    p.ground.w = static_cast<float>(mapAround_);

    // the sea: the wind its waves are made for, the plane they lie in (east and north at an anchor near the eye), its
    // clock. The sea's wind follows the weather's as a sea builds and calms, over minutes (a wind's gust or a turn of
    // the weather did not remake the whole sea at once), and its spectrum is made again as it goes, in steps too small
    // to see: the same waves, each one a little higher or lower, turned a little.
    if (seaShown_) {
        const double from = vsg::radians(weather_.windFromDeg);
        const double speed = seaWindOverride_ >= 0.0 ? seaWindOverride_ : weather_.windMs > 0.5 ? weather_.windMs : kSeaWindDefault;
        const vsg::dvec2 target(-std::sin(from) * speed, -std::cos(from) * speed); // (towards: east, north)
        const bool first = !seaAnchored_;
        seaWind_ = first ? target : seaWind_ + (target - seaWind_) * (1.0 - std::exp(-dt / kSeaWindLag));
        const double windMs = std::clamp(vsg::length(seaWind_), 2.0, 30.0);
        const double windAngle = vsg::length(seaWind_) > 1e-6 ? std::atan2(seaWind_.y, seaWind_.x) : 0.0; // from east towards north
        double turned = std::abs(windAngle - seaMadeAngle_);
        turned = std::min(turned, 2.0 * 3.141592653589793 - turned);
        const bool remake = first || std::abs(windMs - seaMadeMs_) > 0.05 || turned > vsg::radians(0.5);
        if (remake) {
            seaMadeMs_ = windMs;
            seaMadeAngle_ = windAngle;
        }
        if (first || vsg::length(surface - seaAnchor_) > kSeaReanchor) {
            const double lat = vsg::radians(lla.x), lon = vsg::radians(lla.y);
            seaAnchor_ = surface;
            seaU_ = vsg::dvec3(-std::sin(lon), std::cos(lon), 0.0);                                             // east
            seaV_ = vsg::dvec3(-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat)); // north
            seaAnchored_ = true;
        }
        seaSpectrumSwitch_->setAllChildren(remake || seaFrames_ <= 2);
        seaClock_ = std::fmod(seaClock_ + dt, kSeaPeriod);
        // JONSWAP for the wind over the open ocean's fetch (Hasselmann et al. 1973, as GodotOceanWaves sets it)
        const double g = 9.81;
        const double fetch = std::min(kSeaFetch, kSeaFullyDeveloped * seaMadeMs_ * seaMadeMs_ / g);
        const double alpha = 0.076 * std::pow(seaMadeMs_ * seaMadeMs_ / (fetch * g), 0.22);
        const double peak = 22.0 * std::cbrt(g * g / (seaMadeMs_ * fetch));
        // whitecaps from about 4 m/s (Beaufort 3), the foam GodotOceanWaves' "foam amount" makes, per frame
        const double foamAmount = std::clamp((windMs - 4.0) / 2.0, 0.0, 5.0);
        // (a frame's step at most a 30 Hz frame's: a stalled frame - a screenshot written - flooded the sea with foam)
        const double foamDt = std::min(dt, 1.0 / 30.0);
        const double grow = foamAmount * 7.5 * foamDt, decay = std::max(0.5, 10.0 - foamAmount) * 1.15 * foamDt;
        p.ocean = v4(seaClock_, seaMadeMs_, seaMadeAngle_, shown_ ? 1.0 : 0.0);
        p.oceanSpectrum = v4(alpha, peak, 1000.0, 0.25);
        p.oceanSpectrum2 = v4(0.2, 1.0, 0.5, grow);
        // Cox & Munk's slopes for a clean sea: the variance along the wind and across it, as it lies along the plane's
        // axes (east, north)
        const double up = 0.00316 * windMs, cross = 0.003 + 0.00192 * windMs;
        const double c2 = std::cos(windAngle) * std::cos(windAngle), s2 = 1.0 - c2;
        p.oceanFoam = v4(decay, up * c2 + cross * s2, up * s2 + cross * c2, 0.0);
        const double ra = 1.0 / ellipsoid.radiusEquator(), rb = 1.0 / ellipsoid.radiusPolar();
        const vsg::dvec3 o(eyeEcef.x * ra, eyeEcef.y * ra, eyeEcef.z * rb);
        p.oceanEye = v4(o.x, o.y, o.z, vsg::dot(o, o) - 1.0);
        p.oceanRadii = v4(ra, ra, rb, -0.5 * seaFloorDepth()); // (halfway down to the terrain's sea floor)
        p.oceanU = v4(seaU_.x, seaU_.y, seaU_.z, 0.0);
        p.oceanV = v4(seaV_.x, seaV_.y, seaV_.z, 0.0);
        const vsg::dvec3 fromAnchor = eyeEcef - seaAnchor_;
        const double along = vsg::dot(fromAnchor, seaU_), across = vsg::dot(fromAnchor, seaV_);
        for (std::uint32_t c = 0; c < kSeaCascades; ++c) {
            const double tile = kSeaTiles[c];
            const double bandFrom = c == 0 ? 0.0 : kSeaBandStart * 2.0 * 3.141592653589793 / tile;
            const double bandTo = c + 1 < kSeaCascades ? kSeaBandStart * 2.0 * 3.141592653589793 / kSeaTiles[c + 1] : 1e9;
            p.cascade[c] = v4(tile, 0.0, bandFrom, bandTo);
            p.cascadePhase[c] = v4(along / tile - std::floor(along / tile), across / tile - std::floor(across / tile), c == 0 ? along : 0.0,
                                   c == 0 ? across : 0.0);
        }
        // the sun the terrain's haze is lit by where the camera looks (viewer_main's world::setDaylight, its overcast at the
        // ground there): the sea hazed as the land beside it is
        const Daylight lit = daylightAt(toSunEcef, normal, overcast(weather_, groundM));
        p.oceanHaze = v4(lit.sunColour.x * lit.sun, lit.sunColour.y * lit.sun, lit.sunColour.z * lit.sun, 0.0);
        // and the light it gives level ground (VSG's Phong: the sun on it, the sky's from above, the ambient)
        const vsg::vec3 level = lit.sunColour * (lit.sun * static_cast<float>(std::max(vsg::dot(normal, sun), 0.0))) + lit.skyColour * lit.sky +
                                lit.ambientColour * lit.ambient;
        p.oceanLight = v4(level.x, level.y, level.z, 0.0);
        // (Hs from Pierson & Moskowitz's, which the fetch cap holds the spectrum to: 0.0246 U^2)
        p.oceanGrid = v4(seaGridCell_, kSeaGridMargin, peak * peak / g, 0.0246 * seaMadeMs_ * seaMadeMs_);

        // the shore round the eye: the map made on a worker thread taken when it is done, another asked for when the
        // eye has moved on (and is low enough to see a shore's surf)
        if (shoreJob_.valid() && shoreJob_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            ShoreMap made = shoreJob_.get();
            std::memcpy(shoreData_->dataPointer(), made.distance.data(), made.distance.size());
            shoreData_->dirty();
            shoreCentre_ = made.centre;
            shoreEast_ = made.east;
            shoreNorth_ = made.north;
            shoreValid_ = made.any;
        }
        const bool shoreMade = vsg::length(shoreCentre_) > 1.0;
        if (shoreQuery_ && !shoreJob_.valid() && lla.z < kShoreMaxAltitude && (!shoreMade || vsg::length(surface - shoreCentre_) > kShoreMoved)) {
            const double lat = vsg::radians(lla.x), lon = vsg::radians(lla.y);
            const vsg::dvec3 E(-std::sin(lon), std::cos(lon), 0.0);
            const vsg::dvec3 N(-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat));
            const double a = ellipsoid.radiusEquator(), b = ellipsoid.radiusPolar();
            shoreJob_ = std::async(std::launch::async, [query = shoreQuery_, surface, E, N, a, b] {
                ShoreMap m;
                m.centre = surface;
                m.east = E;
                m.north = N;
                m.distance = shoreDistances(query, surface, E, N, a, b, m.any);
                return m;
            });
        }
        const vsg::dvec3 fromShore = eyeEcef - shoreCentre_;
        p.shore0 = v4(fromShore.x, fromShore.y, fromShore.z, 0.5 * kShoreSize * kShoreSpacing);
        p.shoreEast = v4(shoreEast_.x, shoreEast_.y, shoreEast_.z, shoreValid_ && lla.z < kShoreMaxAltitude * 1.5 ? 1.0 : 0.0);
        p.shoreNorth = v4(shoreNorth_.x, shoreNorth_.y, shoreNorth_.z, kShoreStep);
    }
    std::memcpy(params_->dataPointer(), &p, sizeof(Params));
    params_->dirty();

    lastViewProjection_ = projection * view;
    haveLast_ = true;
}

} // namespace fsim::world
