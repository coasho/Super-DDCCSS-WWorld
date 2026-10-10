#pragma once

#include "core/Span.h"
#include "sim/VehicleState.h"
#include "world/VehicleVisuals.h"

#include <vsg/all.h>

#include <array>
#include <cstddef>
#include <deque>
#include <memory>
#include <vector>

namespace fsim::world {

/// The vapour the air makes round an aircraft (design 8.2 "Effects"): a port
/// of aeronautic's wing vapour (@aeronautic/wing-vapor, MIT; see
/// WingVapour.cpp). Nothing is drawn to a shape. The pressure round the
/// aircraft is worked out from its planform (the model's manifest: wing,
/// strakes, canard), its airspeed, angle of attack, sideslip and roll rate -
/// the tip vortices trailing in the air it flew through, the leading-edge
/// vortices over a swept wing or a strake and their bursting, the suction over
/// the wing's upper surface ended by a shock once the flow over it is
/// supersonic, the transonic pocket round the fuselage (the vapour cone) - the
/// day's moist air is expanded through it on the moist adiabat, and cloud is
/// drawn wherever the air passes its dew point: a box per aircraft whose
/// pixels march the eye's ray through that field, lit by the sun and the sky,
/// ending where the vehicles' depth (setSolid) says the airframe is.
class WingVapour {
public:
    /// The day's air: what is drawn depends on it.
    struct Air {
        double temperatureK = 288.15, pressurePa = 101325.0, densityKgM3 = 1.225, soundMs = 340.3;
        double mixingRatio = 0.0, relativeHumidity = 0.0;
    };
    /// The International Standard Atmosphere at that height, warmer by the
    /// day's offset, holding vapour at that relative humidity (0 .. 1).
    static Air moistAir(double altitudeM, double relativeHumidity, double temperatureOffsetK);
    /// The humidity drawn with when the world gives none: aeronautic's default
    /// day's, 85 %.
    static double defaultHumidity(double altitudeM);
    /// Liquid water (kg/m^3) in a parcel of the air expanded to this share of
    /// its pressure (0 .. 1) on the moist adiabat, with this much more vapour
    /// than the day's (1: the same).
    static double liquidWater(double pressureRatio, const Air& air, double humidityScale = 1.0);
    /// The least pressure deficit (1 - p / p0) that condenses anything in the
    /// moistest air the day's patches hold; 1 when nothing can.
    static double saturationDeficit(const Air& air);

    /// What is flying: its true airspeed (m/s), angle of attack, sideslip
    /// (rad; + the wind from the right) and roll rate (rad/s; + right wing down).
    struct Flight {
        double airspeedMs = 0.0, alphaRad = 0.0, betaRad = 0.0, rollRateRadS = 0.0;
    };
    /// The pressure deficit the steady flight makes at a point in the body
    /// frame (m: x forward, y right, z down; the tip vortices trailing straight
    /// back) round the aircraft of this shape: what decides whether the air
    /// there is cloud. For the tests and the docs.
    static double deficit(const VehicleVisuals::Shape& shape, const Flight& flight, const Air& air, const vsg::dvec3& body);
    /// The box round everything that can fog, in the body frame; false when
    /// nothing can.
    static bool bounds(const VehicleVisuals::Shape& shape, const Flight& flight, const Air& air, vsg::dvec3& lo, vsg::dvec3& hi);

    WingVapour(std::size_t slots, vsg::ref_ptr<const vsg::Options> options);
    ~WingVapour();

    /// The group the vapour is drawn under (empty until build()): its image laid on the frame.
    vsg::ref_ptr<vsg::Node> node() const { return root_; }
    /// Recorded before the frame's render pass (render::Viewer::setScene's beforePass): the march.
    vsg::ref_ptr<vsg::Node> compute() const { return compute_; }
    /// Makes the drawing, once the scene is: the rays end where the vehicles
    /// are solid - their depth drawn before the frame
    /// (render::Viewer::depthPrepass) at the window's size `width` x `height`. False when the shaders did not build.
    /// `exhaust`: the engines' exhausts (VehicleVisuals::exhaustData()), marched in the same pass; null for none.
    /// `blueNoise`: the dither the rays are marched with (loadBlueNoise()); null for a flat one. `sceneColour`: the
    /// scene as drawn (render::Viewer::sceneColour), which the exhausts' heat haze shows bent - the composite is then
    /// drawn in the pass over the scene (Viewer::setScene's overScene).
    bool build(vsg::ref_ptr<vsg::ImageInfo> depth, std::uint32_t width, std::uint32_t height, const VehicleVisuals::Compiler& compiler,
               const Exhaust* exhaust = nullptr, vsg::ref_ptr<vsg::ImageInfo> blueNoise = {}, vsg::ref_ptr<vsg::ImageInfo> sceneColour = {});
    bool valid() const noexcept { return static_cast<bool>(state_); }
    /// Built without a compiler: call once the scene it is part of is compiled (its passes on until then, so they
    /// are compiled; off after, until update() has something to draw).
    void compiled();
    /// Whether the vapour shown this frame reads that depth.
    bool wantsSolid() const noexcept { return wantsSolid_; }
    /// The window's size (px), where the vapour reads that depth.
    void setScreen(std::uint32_t width, std::uint32_t height);
    /// Off: nothing drawn, the trails forgotten.
    void setVisible(bool on);
    /// The world's air: its sea-level temperature (K) and humidity (0 .. 1;
    /// 0: none given, defaultHumidity()).
    void setAir(double seaLevelK, double humidity) noexcept;

    /// Per frame, after VehicleVisuals::update(): `states` as drawn, `alive`
    /// per slot, the camera (its view and projection matrices and eye, ECEF), the sun (ECEF,
    /// towards it) and the daylight (0 night .. 1 day).
    void update(Span<const sim::VehicleState> states, const std::vector<unsigned char>& alive, const VehicleVisuals& visuals,
                const vsg::dmat4& view, const vsg::dmat4& projection, const vsg::dvec3& eye, const vsg::dvec3& sun, float daylight);

    struct Model;   ///< a model's wing, as the field reads it
    struct History; ///< a vehicle's path through the air

private:
    static constexpr double kMaxTrail = 400.0; ///< m of trail kept at most
    struct Slot {
        std::vector<double> key; ///< what its model was made from
        std::unique_ptr<Model> model;
        std::unique_ptr<History> history;
        double lastTime = -1.0, rollRate = 0.0, keep = kMaxTrail;
        std::vector<vsg::vec4> table; ///< the condensation table, for the air it was made for
        vsg::dvec3 tableAir;
    };

    vsg::ref_ptr<vsg::Group> root_, compute_;
    vsg::ref_ptr<vsg::Switch> computeSwitch_, drawSwitch_;
    vsg::ref_ptr<vsg::Dispatch> dispatch_;
    vsg::ref_ptr<vsg::vec4Array> data_, screen_, flames_;
    const Exhaust* exhaust_ = nullptr;
    vsg::ref_ptr<vsg::StateGroup> state_;
    vsg::uivec2 alloc_{1, 1};   ///< the images' size (the screen's)
    vsg::vec2 window_{1.0f, 1.0f};
    std::vector<Slot> slots_;
    double seaLevelK_ = 288.15, humidity_ = 0.0;
    bool visible_ = true, wantsSolid_ = false;
};

} // namespace fsim::world
