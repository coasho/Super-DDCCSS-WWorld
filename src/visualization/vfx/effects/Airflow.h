#pragma once

#include "foundation/core/Span.h"
#include "fsim/VehicleState.h"
#include "visualization/scene/Blackbody.h"
#include "visualization/scene/Clouds.h"
#include "visualization/vfx/effects/Cocip.h"
#include "visualization/scene/VehicleVisuals.h"

#include <vsg/all.h>

#include <array>
#include <cstddef>
#include <filesystem>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

namespace fsim::world {

/// Contrails (design 8.2 "Effects"): the contrail cirrus prediction model CoCiP as pycontrails has it (world/Cocip,
/// Apache-2.0), drawn as the plume it describes.
///
/// - Whether one forms: the Schmidt-Appleman criterion, from the air at the aircraft's height (the world's standard
///   atmosphere and humidity), its fuel flow and an engine efficiency (pycontrails' 0.3).
/// - Behind the aircraft: the wake vortex pair carries the plume down and sublimates some of its ice (CoCiP's
///   parametric downwash and survival fraction); in air supersaturated with respect to ice it persists, spreads with
///   the shear and turbulence, its ice settles and mixes with the air and its particles are lost - one CoCiP state a
///   sample along the trail, stepped as it ages; in drier air it is gone in the vortex phase.
/// - Drawn: each part of the trail a Gaussian section eroded by takram's cloud noises into a turbulent edge, its
///   extinction kept on average, marched along each pixel's ray; lit by the clouds' atmosphere, phase function and
///   multiple scattering, and mapped as they are.
///
/// The wake, which CoCiP takes as instantaneous, is drawn as the wake-vortex literature has it: the jets roll up into
/// the pair within some ten spans; the pair (b0 = pi b / 4 apart) carries the primary wake, two tubes of ice, down at
/// its initial speed Gamma0 / (2 pi b0) to CoCiP's descent, leaving the secondary wake, a curtain of ice, above it
/// (kSecondaryShare of the ice, kept at the flight level while the air there is ice-supersaturated - Sussmann and
/// Gierens 1999); the Crow instability bends the tubes (wavelength 8.6 b0, its plane 48 degrees from the horizontal,
/// growing at 0.83 w0 / b0 - Crow 1970) until they touch at the lifespan Crow and Bate's law gives from CoCiP's
/// normalised dissipation rate (1976), and pinch into rings; over another lifespan the rings dissolve into CoCiP's
/// plume. The jets' ice is seen from about 0.15 s of age (an ATTAS's contrail 25-35 m behind its engines); through
/// the vortex phase it is CoCiP's initial ice (initial_iwc) for the jets' size as drawn - their water spread over their
/// growing section - rather than over the vortex's whole plume at once, which in air below ice saturation left none
/// to see. tests/visualization/test_contrail_observed.cpp holds it against observed contrails. Fuel flow, which the state does not
/// carry, is the thrust's power over the fuel's energy at pycontrails' engine efficiency. The world gives no wind
/// profile: the shear is a typical upper-tropospheric 0.004 /s, its part across the track as the world's wind lies to
/// it. The world's humidity is relative to water (WMO); none given, a typical day's (typicalHumidityOverIce: short
/// contrails where the Schmidt-Appleman criterion holds, none persisting). A contrail is dropped once it can no longer
/// be seen (its optical depth under 0.02) or CoCiP ends it, and at most kSources trails of kSamples samples are kept.
///
/// A missile's rocket (its model's manifest names a `rocket`) lays a smoke trail instead, drawn as the plume part of
/// the same trail - no wake of its own worth drawing - its smoke from what the motor burns (smokeExtinction): its
/// primary smoke the propellant's particles, its secondary smoke the HCl in an ammonium perchlorate propellant's
/// exhaust taking up the air's water into droplets where the air is humid enough; spread by the air's turbulence and
/// shear as CoCiP spreads a plume (cocip::disperse), carried by the wind, gone once it can no longer be seen. Where the
/// air is cold enough its exhaust's water freezes as a jet's does, into an ice contrail in the same trail
/// (iceFormation: the Schmidt-Appleman criterion for the motor's exhaust): CoCiP's initial ice over the trail's section,
/// on the exhaust's particles, then CoCiP's plume - sublimating in dry air, persisting and spreading where the air is
/// supersaturated over ice.
class Airflow {
public:
    /// `shaders`: the clouds' shader folder (assets/clouds/shaders); `tables`: the atmosphere's (Clouds::atmosphere()).
    /// `coarseShading`: the device can shade a draw at a coarser rate than its pixels (render::Viewer::coarseShading).
    Airflow(std::size_t slots, vsg::ref_ptr<const vsg::Options> options, const std::filesystem::path& shaders, const AtmosphereTables& tables,
            bool coarseShading = false);

    /// False when the shaders did not build (nothing is drawn).
    bool valid() const noexcept { return static_cast<bool>(state_); }
    vsg::ref_ptr<vsg::Node> node() const { return root_; }
    void setScreen(std::uint32_t width, std::uint32_t height);

    /// Off: nothing drawn, and the trails are forgotten.
    void setVisible(bool on);
    bool visible() const noexcept { return visible_; }

    /// The world's air: its sea-level temperature (K) and pressure (Pa), its relative humidity over water (0..1; 0
    /// none given) and its wind (from, deg; m/s).
    struct Air {
        double seaLevelK = 288.15, seaLevelPa = 101325.0, humidity = 0.0, windFromDeg = 0.0, windMs = 0.0;
    };
    void setAir(const Air& air) noexcept { air_ = air; }
    /// The air at a height in that world (the standard atmosphere from the sea level's; the humidity's convention).
    static cocip::Air airAt(const Air& air, double altitudeM);
    /// The humidity over ice of a typical day at a height, for a world that gives none (an assumption: no
    /// persistent contrails - the upper troposphere short of ice saturation, the stratosphere dry).
    static double typicalHumidityOverIce(double altitudeM);
    /// The shear the contrails spread with (1/s): typical of the upper troposphere.
    static constexpr double kShear = 0.004;

    /// The aircraft in a slot, from its definition (io::readAircraftMetrics): its wingspan (m) and its mass without
    /// fuel (kg). 0: not known - then its model's span and a mass of 120 kg per square metre of span.
    void setAircraft(std::size_t slot, double wingspanM, double massWithoutFuelKg);

    /// Per frame, after VehicleVisuals::update(): `states` as drawn, `alive` per slot, the camera (view and
    /// projection, the eye ECEF), the ellipsoid and the sun (ECEF, towards it).
    void update(Span<const sim::VehicleState> states, const std::vector<unsigned char>& alive, const VehicleVisuals& visuals,
                const vsg::dmat4& projection, const vsg::dvec3& eye, const vsg::EllipsoidModel& ellipsoid, const vsg::dvec3& sun);

    /// Samples kept per trail (the oldest half thinned when full), trails kept.
    static constexpr std::size_t kSamples = 256, kSources = 32;
    /// The ages (s) the young trail is drawn at, between the samples laid (Airflow::update)
    static constexpr double kYoungAges[] = {0.03, 0.06, 0.09, 0.12, 0.15, 0.18, 0.21, 0.24, 0.27, 0.3,
                                            0.36, 0.43, 0.52, 0.62, 0.75, 0.9, 1.08, 1.3, 1.55};
    /// Samples drawn per trail: those laid, the head, and one at each young age
    static constexpr std::size_t kDrawn = kSamples + std::size(kYoungAges);
    /// The secondary wake's share of the young contrail's ice: a B747-400's at 50 s, its optical depth 0.24 against
    /// the primary wake's 1.18 over equal sections (Sussmann and Gierens 1999, in Schumann et al. 2017, 3.2.4).
    static constexpr double kSecondaryShare = 0.24 / (0.24 + 1.18);

    /// A point of a trail: where and when it was laid, the wake that carries it, CoCiP's state as it ages.
    struct Sample {
        vsg::dvec3 at;          ///< where it was emitted, carried by the wind (ECEF)
        double birth = 0.0;     ///< s
        double flightAltitude = 0.0;
        double vortexTime = 0.0;///< s: the vortex phase's length
        double fuelDist = 0.0;  ///< kg of fuel burnt per metre
        double youngN = 0.0;    ///< its ice particles per metre before the descent
        // the wake: the pair's spacing (b0), descent and its speed, its lifespan to linking (Crow and Bate), the
        // time it takes to roll up; the jets' half spread and radius; the distance flown to here
        double separation = 0.0, dzMax = 0.0, descentSpeed = 0.0, linkTime = 0.0, rollUp = 0.0;
        double jetHalf = 0.0, jetRadius = 0.0, flown = 0.0;
        double jetWidth = 0.0, jetDepth = 0.0;
        cocip::Plume plume;     ///< CoCiP's state
        cocip::Shear shear;     ///< the shear it spreads with
        double lastStep = 0.0;
        bool gapBefore = false; ///< not joined to the one before it
        vsg::dvec3 up;          ///< the aircraft's up (its body's -z, ECEF) as it laid it: the wake's frame, banked with it
        /// A rocket's smoke, not ice: `plume` its section only (spread by cocip::disperse), these its extinction per
        /// metre (m), kept as it spreads - its particles and droplets neither fall nor evaporate in the model
        bool smoke = false;
        double smokePrimaryE = 0.0, smokeSecondaryE = 0.0;
        /// and its ice contrail (where its exhaust's water froze): CoCiP's plume, stepped as a jet's (alive: there is ice)
        cocip::Plume ice;
        /// A fire's smoke (an engagement's Fire event: layFire), not white: soot of this albedo, and, while young, the
        /// flame - the soot at its temperature, glowing as much as it absorbs (Kirchhoff)
        bool fire = false;
        double albedo = 1.0, fireScale = 1.0; ///< (its fire's size: fireScale)
        /// fuel mist: what a leak throws out that does not burn - droplets that scatter (albedo 1), evaporating
        bool mist = false;
        /// a fire's far trail only (world::FlowVolumes draws its flame and near smoke): its soot the share the near
        /// field's has lost by its age, 1 - exp(-handOver age) (1/s; 0: all of it, and its flame)
        double handOver = 0.0;
    };
    /// What lays a trail: the aircraft, its altitude, its jets (how far they lie either side of its middle; the
    /// widest's radius), the shear the trail spreads with.
    struct Emitter {
        cocip::Aircraft aircraft;
        double altitude = 0.0, jetHalf = 0.0, jetRadius = 0.5;
        cocip::Shear shear;
    };
    /// A sample laid at `now` where the jets are (`at`, ECEF): CoCiP's wake vortex and first plume there.
    static Sample lay(const Emitter& e, const cocip::AirAt& air, const vsg::dvec3& at, double now);
    /// Its CoCiP state stepped to `now`, as often as its age asks.
    static void age(Sample& s, const cocip::AirAt& air, double now);
    /// What is drawn of a sample at `now` (assets/clouds/shaders/contrail.glsl): each part's extinction per metre of
    /// trail (m: the integral of its extinction over its section, the observations' "total extinction") - the primary
    /// wake's two tubes together, the secondary wake's curtain, CoCiP's plume; the plume's section (m^2) and its
    /// centre below the flight path (m); the wake: b0, the tubes' half spacing, the pair's descent, the Crow wave's
    /// amplitude, how far it has pinched into rings (0..1), a tube's sigma (m); the wake's share against the plume's
    /// (1 young .. 0).
    struct Drawn {
        double primaryE = 0.0, secondaryE = 0.0, plumeE = 0.0;
        double plumeSyy = 0.0, plumeSzz = 0.0, plumeSyz = 0.0, plumeDrop = 0.0;
        double b0 = 0.0, halfSpacing = 0.0, descent = 0.0, crow = 0.0, rings = 0.0, tube = 0.0, young = 0.0;
        /// the plume's albedo (1: ice) and its flame: the soot's temperature (K, 0: none) - a fire's (flameAt)
        double albedo = 1.0, flameK = 0.0;
        double total() const noexcept { return primaryE + secondaryE + plumeE; }
    };
    static Drawn drawn(const Sample& s, const cocip::AirAt& air, double now);

    /// A vehicle's component on fire (an engagement's Fire event): from `now` its fire streams from where the
    /// component is (its model manifest's `component` line) - the flame, then its black smoke - while `rateKgS` of
    /// fuel burns there; 0 puts it out. A fire on a vehicle no longer there goes out with it.
    void ignite(std::size_t slot, const std::string& component, double rateKgS, double now, double mistKgS = 0.0);
    void clearFires() { fires_.clear(); }
    /// The world's clock for the next update (s): given, the trails are laid and aged by it; not given (NaN), by the
    /// first live vehicle's own - which stops with a vehicle that has struck the ground, and every trail with it.
    void setClock(double now) noexcept { clock_ = now; }
    /// What each slot's engines burn now, kg/s, as the simulation has it (a sample's `derived` from
    /// ipc::kDerivedFuelFlowKgS): a contrail's fuel and a motor's smoke are laid from it. A slot it has none for
    /// (all 0, or past the end: a recording from before it was said) is laid from its thrust - a jet's at
    /// pycontrails' engine efficiency, a motor's at its class's specific impulse.
    void setFuelFlows(const std::vector<std::array<double, sim::VehicleState::kMaxEngines>>& flows) { flows_ = flows; }
    /// The overall efficiency a jet flies at (its thrust's power over its fuel's heat) where its fuel flow is known -
    /// what the Schmidt-Appleman criterion takes - else pycontrails' 0.3. Kept to what an engine can be (0.1 - 0.5).
    static double engineEfficiency(double thrustN, double speedMs, double fuelFlowKgS);
    /// The fires' far trails only, from where a near field drawn elsewhere (world::FlowVolumes) thins at this rate (1/s:
    /// its smoke's e-fold; 0 - the default - the whole fire here: its flame, its smoke, its mist).
    void setFireHandOver(double perSecond) noexcept { fireHandOver_ = perSecond; }
    /// A fuel fire in the airstream (held to the drone hits photographed at China Lake and Point Mugu): its soot
    /// kerosene's (kFireSootYield of the fuel, Tewarson's for a sooty liquid fuel; 7.5 m^2/g, fresh soot's absorption,
    /// Bond and Bergstrom 2006; albedo 0.2) laid along the trail from a section kFireWidth across (estimates: a holed
    /// tank's spray in the wake); the flame the young part of it, its soot at kFlameK (a black body of a jet fire's
    /// surface emissive power, 250 kW/m^2 - the middle of the 200 - 350 CCPS gives for hydrocarbon jet fires; the
    /// airstream mixes it as a jet's, not a sooty pool fire's 20 - 130) for kFlameS, cooling over kFlameCoolS (estimates: some twenty metres of flame behind an
    /// aircraft at 200 m/s, as photographed).
    static constexpr double kFireSootYield = 0.04, kSootAbsorption = 7500.0, kSootAlbedo = 0.2, kFireWidth = 1.5;
    static constexpr double kFlameK = 1450.0, kFlameS = 0.1, kFlameCoolS = 0.05;
    /// ... at kFireRateKgS: a larger leak's flame longer and its section wider as the square root of its rate (a fire in
    /// the airstream's length as (m / (rho U))^1/2 - an estimate's form, no published correlation); a tank shredded by
    /// a close burst, dumping hundreds of kilograms a second, a fireball's streak attached to it, the QF-4B's
    static constexpr double kFireRateKgS = 2.0;
    /// A leak's unburnt fuel: droplets of some 100 um (a torn tank's spray atomized by the airstream - 50 to 150 um: an
    /// estimate), 3 / (2 rho d / 2) = 37.5 m^2 of scattering per kg, white; evaporating in kMistLifeS (an e-fold:
    /// kerosene's droplets that size in the cold air at altitude, an estimate) - the QF-4B's white streaming
    static constexpr double kMistExtinction = 37.5, kMistLifeS = 2.0;
    /// The scale of a fire at this rate against kFireRateKgS's (1 at it).
    static double fireScale(double rateKgS) { return std::sqrt(std::max(rateKgS, 0.05) / kFireRateKgS); }
    /// The flame's radiating temperature (K, 0: none) at an age - the black body of its mean emissive power; drawn
    /// fluctuating (kFlicker) about the mean that gives it; a fire of this scale's (fireScale) burning longer.
    static double flameAt(double age, double scale = 1.0);
    /// A fire's sample laid at `now` where the component is (`at`, ECEF): its soot per metre at the airspeed.
    static Sample layFire(double rateKgS, double airspeedMs, double altitude, const cocip::Shear& shear, const cocip::AirAt& air,
                          const vsg::dvec3& at, double now);
    /// A missile's motor as its trail sees it: its propellant's smoke class (0 minimum, 1 reduced, 2 aluminised, the
    /// manifest's), its thrust (N), the missile's speed (m/s) and its nozzle's exit radius (m).
    struct Smoke {
        int kind = 1;
        double thrustN = 0.0, speed = 0.0, exitRadius = 0.05;
        /// The propellant it burns now, kg/s, as the simulation has it (0: not told - its thrust over its class's
        /// specific impulse, as before the simulation said).
        double flowKgS = 0.0;
        /// kg of exhaust a second: the simulation's figure, else the thrust's.
        double massFlow() const;
    };
    /// The smoke it lays per metre of trail in this air: each part's extinction per metre (m, the integral of its
    /// extinction over the trail's section, as Drawn's) - the primary smoke, the secondary.
    struct SmokeExtinction {
        double primary = 0.0, secondary = 0.0;
    };
    static SmokeExtinction smokeExtinction(const Smoke& m, const cocip::Air& air);
    /// The relative humidity over water (0..1) above which an ammonium perchlorate motor's HCl takes up the air's
    /// water into a visible aerosol, at a temperature (K); and the HCl's mass fraction in droplets at equilibrium with
    /// air at a relative humidity.
    static double secondarySmokeOnset(double temperatureK);
    /// Whether its exhaust's water freezes into a contrail in this air: the Schmidt-Appleman criterion for a rocket's
    /// exhaust (its water, and the heat it brings the plume, per kg of propellant burnt - rocketWater, rocketHeat).
    static cocip::Formation iceFormation(const Smoke& m, const cocip::Air& air);
    /// kg of water per kg of propellant, its hydrogen burnt in the air; J per kg into the plume (world/Propellants.h)
    static double rocketWater(int smokeClass);
    static double rocketHeat(int smokeClass, double speed, double temperatureK);
    static double acidMassFraction(double relativeHumidity);
    /// A sample of smoke laid at `now` where the motor's exit is (`at`, ECEF).
    static Sample laySmoke(const Smoke& m, double altitude, const cocip::Shear& shear, const cocip::AirAt& air, const vsg::dvec3& at,
                           double now);

private:

    struct Source {
        int slot = -1;
        std::vector<Sample> samples; ///< oldest first
        double lastCommit = -1e9;
        bool forming = false, used = false;
        vsg::dvec3 headAt;          ///< where the jets are now: the trail reaches them, clear until its ice has formed
        vsg::dvec3 headUp;          ///< the aircraft's up now
        double headAltitude = 0.0;
        double phase = 0.0;         ///< the trail's own phase of the Crow wave
    };
    Source* source(int slot, bool create);
    struct Fire {
        std::size_t slot = 0;
        std::string component;
        double rateKgS = 0.0, since = 0.0, mistKgS = 0.0;
    };
    std::vector<Fire> fires_;
    double fireHandOver_ = 0.0;              ///< the components on fire, each its own trail (source kFireKey + index)
    static constexpr int kFireKey = 1 << 20;
    /// A missile's slot: its motor's smoke laid while it burns (update's, for a vehicle whose model names a rocket).
    void smokeFrom(std::size_t slot, const sim::VehicleState& state, const VehicleVisuals::Shape& shape, const cocip::AirAt& air, double now);

    vsg::ref_ptr<vsg::Group> root_;
    vsg::ref_ptr<vsg::Switch> switch_;
    vsg::vec2 screen_{1.0f, 1.0f};
    vsg::ref_ptr<vsg::StateGroup> state_;
    vsg::ref_ptr<vsg::MatrixTransform> ribbonAt_;
    vsg::ref_ptr<vsg::Draw> ribbonDraw_;
    vsg::ref_ptr<vsg::Command> shadingRate_; ///< the contrails' shading rate, set before their draw (null: per pixel)
    std::uint32_t coarse_ = 1;               ///< pixels a fragment shades each way
    vsg::ref_ptr<vsg::vec4Array> samples_, frame_;
    SunlitBlackbody blackbody_; ///< a flame's light against the sun's (its table after the samples, for the shaders)
    std::vector<Source> sources_;
    std::vector<vsg::dvec2> aircraft_; ///< per slot: wingspan, mass without fuel
    double clock_ = std::numeric_limits<double>::quiet_NaN();
    std::vector<std::array<double, sim::VehicleState::kMaxEngines>> flows_; ///< per slot: each engine's fuel flow, kg/s
    Air air_;
    bool visible_ = true;
    double lastTime_ = -1.0;
};

} // namespace fsim::world
