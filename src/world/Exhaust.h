#pragma once

#include "sim/VehicleState.h"

#include <vsg/all.h>

#include <cstddef>
#include <vector>

namespace fsim::world {

/// The engines' exhaust (design 8.2 "Effects"), drawn at a model's
/// `fsim:afterburner:<engine>` nodes instead of the flame mesh the model
/// carries - a port of aeronautic's afterburner (MIT; see Exhaust.cpp). The
/// jet is worked out on the CPU from the nozzle, the gas and the air (its
/// expansion, potential core, spreading, shock train, what burns in its
/// shear layer and the soot in it) and its plume marched through on the GPU:
/// soot glowing at its temperature, the radicals' violet, the diamonds, the
/// eddies; up the nozzle from astern the turbine and the pipe glow a dull red
/// dry, the burner's flame fills it lit. A stopped engine shows nothing. The
/// jet runs from its node as the model places it at rest (VehicleVisuals
/// hands its place on), and the node's own geometry gives its size.
///
/// The cost is kept off the CPU: a handful of numbers per engine (ten vec4
/// rows in one storage buffer) and a clock. The march itself (marchFlame) is
/// done by the pass that marches all the gas round the aircraft, the wing
/// vapour's (WingVapour): under its budget of rays, so a nozzle filling the
/// screen costs what a small one does, and composited with the vapour in the
/// order the two lie along each ray.
class Exhaust {
public:
    /// Engines per vehicle slot it can draw.
    static constexpr std::size_t kPerSlot = sim::VehicleState::kMaxEngines;

    Exhaust(std::size_t slots, vsg::ref_ptr<const vsg::Options> options);

    /// False when it could not be set up: models keep their own flames.
    bool valid() const noexcept { return static_cast<bool>(rows_); }

    /// Where engine `k` of slot `slot`'s exhaust hangs (standing in for the
    /// model's own flame there), made on first use and kept, or null past the
    /// limits. Nothing is drawn under it: the gas is marched by the pass that
    /// marches it round the aircraft (WingVapour), from this class's numbers.
    vsg::ref_ptr<vsg::Node> node(std::size_t slot, std::size_t k);

    /// What kind of jet engine it is (the model's manifest, hangar's design):
    /// its bypass ratio and whether it has an afterburner. Unknown, a
    /// fighter's augmented low-bypass turbofan, as drawn before kinds were known.
    struct Kind {
        double bypassRatio = 0.5;
        bool augmented = true;
        /// A missile's solid rocket motor (the manifest's `rocket` line, hangar's [motor]): its plume follows the
        /// thrust (VehicleState::thrustN against `peakThrustN`) instead of a turbine's core speed and burner, and its
        /// particles its propellant's smoke class.
        bool rocket = false;
        enum Smoke { Minimum, Reduced, Aluminised } smoke = Reduced;
        double peakThrustN = 0.0; ///< its boost's thrust (the plume at full pressure); 0 unknown: any thrust is full
        /// A jet engine's ratings, its design's (the manifest's `engine` line): its thrust at full dry power and with
        /// reheat (N; 0: not said - the preset's reheat). Its reheat's temperature is drawn from the thrust it adds.
        double dryThrustN = 0.0, reheatThrustN = 0.0;
    };
    /// The total temperature (K) of an augmented engine's exhaust with its reheat lit, from its own ratings: lit, its
    /// burner heats the same flow at the same pressure, so the thrust goes as the jet's speed - the square root of its
    /// temperature - and T_reheat = T_dry (F_reheat / F_dry)^2 (less the fuel's own mass, kFuelMassShare of the
    /// thrust). T_dry its mixed exhaust's at full dry power: the turbine's exit gas (kTurbineExitK) and its bypass air
    /// (kBypassK) by its bypass ratio. Kept to what a burner at its full reheat reaches (kLeastReheatK .. kMostReheatK:
    /// the least a rating is written for - an engine whose dry exhaust is hotter than the type's adds less thrust for
    /// the same flame - and kerosene's stoichiometric flame less its cooling air; estimates). 0: the ratings do not
    /// say (the preset's).
    static double reheatTotalK(double bypassRatio, double dryThrustN, double reheatThrustN);
    static constexpr double kTurbineExitK = 1050.0, kBypassK = 430.0; ///< at full dry power (estimates: typical of the type)
    static constexpr double kFuelMassShare = 0.04;                    ///< the reheat's fuel in the jet's mass (an estimate)
    static constexpr double kLeastReheatK = 1850.0, kMostReheatK = 2300.0;

    /// What engine `engine` of `state` shows this frame, for a nozzle this
    /// wide and high (half its width and height, m: a two-dimensional
    /// nozzle's are not the same), of this kind. False when it shows nothing (hide the node).
    bool set(std::size_t slot, std::size_t k, double radiusY, double radiusZ, const sim::VehicleState& state, int engine,
             const Kind& kind);
    bool set(std::size_t slot, std::size_t k, double radiusY, double radiusZ, const sim::VehicleState& state, int engine) {
        return set(slot, k, radiusY, radiusZ, state, engine, Kind{});
    }

    /// The clock the eddies move by (s; the simulation's, so a paused or
    /// replayed world shows the same) and the daylight (0 night .. 1 day):
    /// the camera's exposure, by day what the sky leaves it.
    void setFrame(double seconds, float daylight);

    /// Hands this frame's numbers to the GPU (once, after the set() calls).
    void commit();

    /// What a nozzle of this radius shows at this afterburner (0..1) and
    /// dry power (0 idle .. 1 military): the flame's length (m, 0 unlit).
    static double flameLength(double radius, double afterburner);
    /// Dry power from a turbine's core speed (VehicleState::engineN2, %).
    static double dryPower(double n2Percent);
    /// How far it runs, 0 stopped .. 1 at idle and above (core speed, %).
    static double running(double n2Percent);


    /// Its numbers for the GPU: kRows vec4s per engine (slot * kPerSlot + k), the blackbody table, the frame
    /// (time, daylight, exposure); and the march that reads them (GLSL, at bindings 11, 12 and 13): marchFlame().
    static constexpr std::uint32_t kRows = 10;
    vsg::ref_ptr<vsg::vec4Array> rows() const { return rows_; }
    vsg::ref_ptr<vsg::vec4Array> blackbodyTable() const { return blackbody_; }
    vsg::ref_ptr<vsg::vec4Value> frame() const { return frame_; }
    static const char* marchSource();

private:
    struct Flame {
        vsg::ref_ptr<vsg::Group> node;
    };
    std::vector<Flame> flames_; ///< slot * kPerSlot + k
    vsg::ref_ptr<vsg::vec4Array> rows_;   ///< ten per flame: its jet
    vsg::ref_ptr<vsg::vec4Array> blackbody_;
    vsg::ref_ptr<vsg::vec4Value> frame_;  ///< x time (s), y daylight, z exposure
    bool changed_ = false;
};

} // namespace fsim::world
