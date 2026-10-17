#pragma once

#include <vsg/all.h>

#include <array>
#include <filesystem>
#include <functional>
#include <future>
#include <optional>
#include <string>

namespace fsim::world {

/// Bruneton's precomputed atmosphere as takram's three-atmosphere ships it (assets/clouds/atmosphere): each table one
/// image, shared by whatever is lit by it (the clouds, the contrails). Empty when the files are not there.
struct AtmosphereTables {
    vsg::ref_ptr<vsg::ImageInfo> transmittance, scattering, irradiance;
    bool valid() const noexcept { return transmittance && scattering && irradiance; }
    /// The descriptors the shaders' fsim/atmosphere.glsl reads (bindings 6, 7, 8 and 20).
    vsg::Descriptors descriptors() const;
};
AtmosphereTables loadAtmosphereTables(const std::filesystem::path& cloudsDir);
/// takram's two cloud noises (assets/clouds/shape.bin, 128^3 Perlin-Worley; shape_detail.bin, 32^3 Worley), mipmapped,
/// repeating: for whatever else is made of ice the way the clouds are (the contrails). Empty when they are not there.
struct CloudNoise {
    vsg::ref_ptr<vsg::ImageInfo> shape, detail;
    bool valid() const noexcept { return shape && detail; }
};
CloudNoise loadCloudNoise(const std::filesystem::path& cloudsDir);
/// takram's spatio-temporal blue noise (assets/clouds/stbn.bin, 128 x 128 x 64, R8), read texel by texel: a dither
/// with no pattern of its own, for whatever else marches rays (the gas round the aircraft). Null when it is not there.
vsg::ref_ptr<vsg::ImageInfo> loadBlueNoise(const std::filesystem::path& cloudsDir);
/// A shader from the clouds' shader folder, its #include "..." lines replaced by the files they name, `defines`
/// put after its #version; null when it cannot be read.
vsg::ref_ptr<vsg::ShaderStage> loadCloudShader(const std::filesystem::path& shaders, const std::string& name, VkShaderStageFlagBits stage,
                                               const std::string& defines = {});

/// The clouds (design 8.2 "Sky / atmosphere"): takram's volumetric clouds for three.js, @takram/three-clouds
/// (takram-design-engineering/three-geospatial, MIT), ported - their shaders and textures, their defaults and their
/// "high" quality preset - and lit by the precomputed atmosphere they are lit by (Eric Bruneton's, as
/// @takram/three-atmosphere ships its tables). Their files are in assets/clouds; what each changes is marked there.
///
/// - Four layers (takram's CloudLayers.DEFAULT: two low layers of cumulus and stratocumulus and a thin cirrus sheet)
///   over a local weather texture repeated round the globe on a cube-sphere, shaped by a 3D noise and eroded by a
///   finer one and a turbulence; a coverage parameter fills them (Skybolt's coverage modulation).
/// - Lighting: the sun's and the sky's light from the atmosphere at each sample, the optical depth to the sun marched
///   a little way and read from Beer shadow maps beyond (three cascades over 100 km), multiple scattering as octaves,
///   a two-lobed Henyey-Greenstein phase, light bounced off the ground, the powder effect, and the atmosphere's aerial
///   perspective between the eye and the cloud.
/// - Cost: the march runs at a quarter of the window's size each way, one of each 4 x 4 pixels a frame in a Bayer
///   order, and a temporal resolve fills the rest from the last frames (takram's temporal upscaling); the shadow maps
///   are marched at 512 x 512 and resolved over frames the same way.
///
/// Adapted to the viewer, and only so: the cover where the eye is comes from the world's weather (turned into
/// takram's coverage by a curve measured on their own textures); farther off it follows NASA's Blue Marble cloud
/// map, as Skybolt does with its global coverage map (takram draws no views from space); the layers sit on the
/// weather's cloud base; the clouds are drawn over the viewer's frame with three.js's AgX at takram's exposure and
/// depth-tested against it where they start (the scene's depth is not there before the march, so their rays do not
/// end at it); the textures are read at the level a pixel's footprint asks for (mipmapped; takram's noise has none
/// and aliased from far off); their shadow on the ground is read
/// from the shadow maps; their haze and light shafts, which belong to their own atmosphere's sky and ground, are not.
///
/// The sea is drawn with them (under(), after the clouds' shade on the ground), lit by the same atmosphere and shaded
/// by the same shadow maps (assets/clouds/shaders/ocean): GodotOceanWaves' FFT waves (2Retr0, MIT) - a TMA spectrum
/// for the wind in three cascades of 256 x 256, 1987 m, 409 m and 71 m, each a band of it - lit as Bruneton, Neyret &
/// Holzschuch light the sea from its slopes' statistics (Proland, BSD-3), so it is the same sea near and far; where
/// it is and how deep from GEBCO 2026 (tools/data/ocean_map), the imagery seen through the shallows.
class Clouds {
public:
    /// What the world's weather says (sim::EnvironmentState).
    struct Weather {
        double cover = 0.0;          ///< 0..1
        double baseM = 2000.0;       ///< above mean sea level
        double precipitation = 0.0;  ///< 0..1
        double windFromDeg = 0.0;
        double windMs = 0.0;
    };
    /// One of takram's layers: its base and depth (m above the ellipsoid).
    struct Layer {
        double altitudeM = 0.0, heightM = 0.0;
    };
    /// The layers a weather makes: takram's default four, the two low ones on the weather's cloud base.
    static std::array<Layer, 4> layersFor(const Weather& w);
    /// takram's coverage for a cloud cover (0..1 of the sky): the curve measured on their weather and shape textures
    /// (tools/data/cloud_map/coverage_curve.py); 0 for none.
    static double coverageForCover(double cover);
    /// How much of the sun's light an overcast takes away at this height (0 none .. 0.85 under a thick deck): the
    /// light everything is lit with there (world::setDaylight). A scattered sky takes none - its shadows are local.
    static double overcast(const Weather& w, double altitudeM);

    /// `dir`: takram's files (assets/clouds); `earthMap`: the Blue Marble cloud map (one grey channel, 2:1, row 0
    /// north), or none; `seaMap`: GEBCO's sea (tools/data/ocean_map: R the water's depth, log-coded, G its share of the
    /// texel; 2:1, row 0 north), or none - and no sea drawn.
    Clouds(vsg::ref_ptr<const vsg::Options> options, std::filesystem::path dir, vsg::ref_ptr<vsg::Data> earthMap = {},
           vsg::ref_ptr<vsg::Data> seaMap = {});

    /// Into the scene after the terrain and before the vehicles: the clouds' shadow on the ground.
    vsg::ref_ptr<vsg::Node> under() const { return under_; }
    /// Into the scene just after the vehicles: the clouds, and their depth where they are all but opaque.
    vsg::ref_ptr<vsg::Node> over() const { return over_; }
    /// Recorded before the frame's render pass (render::Viewer::setScene's beforePass).
    vsg::ref_ptr<vsg::Node> compute() const { return compute_; }
    /// The atmosphere's tables it is lit by (loaded with it).
    const AtmosphereTables& atmosphere() const noexcept { return atmosphere_; }

    /// Build it for a window of this size; what it makes is compiled with `compiler`.
    using Compiler = std::function<bool(vsg::ref_ptr<vsg::Node>)>;
    void build(std::uint32_t width, std::uint32_t height, Compiler compiler);

    /// The world's weather, or a sky of the viewer's own. None given yet (a viewer waiting for its world), the sky
    /// near the eye is the global cloud map's, as it is seen from far off.
    void setWeather(const Weather& w) {
        given_ = w;
        weatherGiven_ = true;
        weather_ = w;
    }
    /// The weather drawn (update): the one given, or the global map's cover round the eye
    const Weather& weather() const noexcept { return weather_; }
    void setVisible(bool on) { visible_ = on; }
    /// Shown (on, with cloud in the sky).
    bool shown() const noexcept { return shown_; }
    /// The sea, drawn or not (the imagery's own sea alone) whether the clouds are.
    void setSeaVisible(bool on) { seaVisible_ = on; }
    /// The wind the sea's waves are made for (m/s), whatever the weather says; < 0: the weather's.
    void setSeaWind(double ms) { seaWindOverride_ = ms; }
    bool seaShown() const noexcept { return seaShown_; }
    /// Where the land is near the eye, for the sea's shore (the height above the ellipsoid at a latitude and longitude,
    /// radians: land where it is above 0 m - the map set's elevation tiles, the coast burnt into them). Called from a
    /// worker thread; none, and the sea has no surf.
    using GroundQuery = std::function<std::optional<double>(double latitudeRad, double longitudeRad)>;
    void setShoreQuery(GroundQuery query) { shoreQuery_ = std::move(query); }
    void setScreen(std::uint32_t width, std::uint32_t height);

    /// Each frame: the camera (view and projection matrices, the eye in ECEF), the sun (towards it, ECEF), the
    /// ground's height under the eye, the simulation's time.
    void update(const vsg::dmat4& view, const vsg::dmat4& projection, const vsg::dvec3& eyeEcef, const vsg::EllipsoidModel& ellipsoid,
                const vsg::dvec3& toSunEcef, double groundM, double timeS);

    struct Params; ///< the shaders' uniform block (assets/clouds/shaders/fsim/params.glsl)

private:
    vsg::ref_ptr<const vsg::Options> options_;
    std::filesystem::path dir_;
    vsg::ref_ptr<vsg::Data> earthMap_, seaMap_;
    AtmosphereTables atmosphere_;
    vsg::ref_ptr<vsg::Group> under_, over_, compute_;
    vsg::ref_ptr<vsg::Switch> underSwitch_, overSwitch_, computeSwitch_, firstSwitch_, shadowResolveSwitch_, marchSwitch_, resolveSwitch_;
    vsg::ref_ptr<vsg::ubyteArray> params_;
    Weather weather_, given_;
    bool weatherGiven_ = false;
    bool visible_ = true, shown_ = false, built_ = false;
    std::uint64_t frame_ = 0, shownFrames_ = 0;
    vsg::vec2 screen_{1.0f, 1.0f};
    vsg::uivec2 lowAlloc_{1, 1}, fullAlloc_{1, 1};
    vsg::uivec2 lastLow_{0, 0};
    int parity_ = 0;
    std::vector<vsg::ref_ptr<vsg::Dispatch>> lowDispatches_, fullDispatches_;
    // last frame, for the reprojections
    bool haveLast_ = false, shadowHistory_ = false, cloudsHistory_ = false;
    vsg::dmat4 lastViewProjection_;     ///< ECEF -> clip
    vsg::dmat4 lastShadow_[3];          ///< ECEF -> each cascade's clip
    vsg::dvec2 weatherOffset_{0.0, 0.0};
    vsg::dvec3 shapeOffset_{0.0, 0.0, 0.0}, detailOffset_{0.0, 0.0, 0.0};
    double lastTime_ = -1.0;
    // the global map's mean cover round the eye (the world's cover laid out as the map's clouds lie), and where the eye
    // was when it was taken
    double mapAround_ = 0.0;
    vsg::dvec2 mapAroundAt_{1e9, 1e9};
    // the sea: its passes; the plane its waves lie in (tangent at an anchor near the eye: east, north), its wind (towards:
    // east, north, m/s) and the one its spectrum was made for, and its clock
    vsg::ref_ptr<vsg::Switch> seaComputeSwitch_, seaFirstSwitch_, seaSpectrumSwitch_, seaDrawSwitch_;
    bool seaBuilt_ = false, seaVisible_ = true, seaShown_ = false, seaAnchored_ = false;
    std::uint64_t seaFrames_ = 0;
    vsg::dvec3 seaAnchor_, seaU_, seaV_;
    vsg::dvec2 seaWind_;
    double seaMadeMs_ = 0.0, seaMadeAngle_ = 0.0, seaClock_ = 0.0;
    double seaWindOverride_ = -1.0;
    float seaGridCell_ = 0.0f; ///< the sea's grid: a cell's width (0..1 of the screen)
    // the shore round the eye: a map of the distance to it, made on a worker thread as the eye moves
    struct ShoreMap {
        vsg::dvec3 centre, east, north; ///< ECEF: its middle on the ellipsoid, and its axes
        std::vector<std::uint8_t> distance;
        bool any = false;               ///< any land in it
    };
    GroundQuery shoreQuery_;
    std::future<ShoreMap> shoreJob_;
    vsg::ref_ptr<vsg::ubyteArray2D> shoreData_;
    vsg::dvec3 shoreCentre_, shoreEast_, shoreNorth_;
    bool shoreValid_ = false;
};

} // namespace fsim::world
