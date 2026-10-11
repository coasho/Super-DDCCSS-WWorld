#pragma once

#include "fsim/ControlInputs.h"
#include "fsim/InitialConditions.h"
#include "fsim/Property.h"
#include "fsim/VehicleState.h"

#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace fsim::sim {

class GroundProvider;

/// What to load (design 7.2).
struct AircraftSpec {
    std::string name;                 ///< JSBSim aircraft name, e.g. "c172x", "f16"
    std::filesystem::path jsbsimRoot; ///< directory containing aircraft/, engine/, systems/
    /// Directory holding <name>/<name>.xml when it is not <jsbsimRoot>/aircraft
    /// (aircraft of our own: io::AssetResolver::findAircraft). The aircraft's
    /// Engines/ folder is searched before <jsbsimRoot>/engine, as JSBSim does.
    std::filesystem::path aircraftDir = {};
};



/// What a vehicle flies on, as its flight model has it now (docs/flight-autonomy.md,
/// 4.14): fuel - what its tanks hold and could, what its engines burn - or a
/// battery - its charge and capacity, the power it gives. NaN where the model
/// has none (an aircraft with neither, a model that does not say).
struct EnergyOnBoard {
    double fuelKg = std::numeric_limits<double>::quiet_NaN();         ///< in its tanks
    double fuelCapacityKg = std::numeric_limits<double>::quiet_NaN(); ///< they hold, full
    double fuelFlowKgS = std::numeric_limits<double>::quiet_NaN();    ///< its engines burn, now
    double chargeJ = std::numeric_limits<double>::quiet_NaN();        ///< left in its battery
    double chargeCapacityJ = std::numeric_limits<double>::quiet_NaN();
    double powerW = std::numeric_limits<double>::quiet_NaN();         ///< the battery gives, now
    double massKg = std::numeric_limits<double>::quiet_NaN();         ///< the whole vehicle's, now
    bool starved = false; ///< an engine that burns fuel has none left
};

/// Flight-dynamics interface hiding JSBSim (design 7.1, ADR-10).
class FlightModel {
public:
    virtual ~FlightModel() = default;

    /// Load an aircraft and apply initial conditions. Returns false (and logs)
    /// on failure; never throws for data errors.
    virtual bool load(const AircraftSpec& aircraft, const InitialConditions& ic) = 0;

    /// Re-apply initial conditions to the loaded aircraft. Cheap (~1 ms).
    virtual bool reset(const InitialConditions& ic) = 0;

    /// Advance exactly one fixed step `dt()` with the given commands.
    virtual void step(const ControlInputs& inputs) = 0;

    /// Fill `out` with the current state in SI / ECEF metres.
    virtual void state(VehicleState& out) const = 0;

    /// Resolve a property path once (e.g. "velocities/vc-kts"). Invalid paths
    /// return an invalid handle and log at warn level.
    virtual PropertyHandle property(std::string_view path) = 0;

    /// Every numeric property below `prefix` (e.g. "fsim/control"), with its
    /// path relative to it ("pid_attitude/pitch/kp"): data the aircraft
    /// carries for the platform. Empty when there is none.
    virtual std::vector<std::pair<std::string, double>> properties(std::string_view prefix) const {
        (void)prefix;
        return {};
    }

    virtual double dt() const noexcept = 0;
    virtual bool loaded() const noexcept = 0;

    // --- Environment and disturbances (design 9.4, 9.5). Defaults are no-ops
    // so a model that cannot honour a channel simply ignores it.

    /// Steady wind at the vehicle, NED m/s (the direction the air moves).
    virtual void setWindNed(double north, double east, double down) { (void)north; (void)east; (void)down; }
    /// Turbulence intensity 0 (off) .. 1 (severe) with the reference wind speed at 20 ft AGL.
    virtual void setTurbulence(double intensity, double windSpeed20ftMs) { (void)intensity; (void)windSpeed20ftMs; }
    /// Sea-level temperature (K) and pressure (Pa) of the standard atmosphere.
    virtual void setAtmosphere(double temperatureSeaLevelK, double pressureSeaLevelPa) { (void)temperatureSeaLevelK; (void)pressureSeaLevelPa; }
    /// External force (N) and moment (N m) in the body frame, applied at the
    /// centre of gravity, held until changed.
    virtual void setExternalForceBody(const double forceN[3], const double momentNm[3]) { (void)forceN; (void)momentNm; }
    /// Seed the model's own random processes (turbulence, dispersions).
    virtual void seed(std::uint64_t value) { (void)value; }
    /// Effectors beyond ControlInputs (speedbrake, pitch trim); NaN fields are left alone.
    virtual void setEffectors(const EffectorInputs& effectors) { (void)effectors; }
    /// What it flies on now: its fuel or its battery (none known by default).
    virtual EnergyOnBoard energy() const { return {}; }
    /// What engine `engine` burns now, kg/s: a turbine's or a piston's fuel, a rocket motor's propellant (0: none, or
    /// no such engine).
    virtual double engineFuelFlowKgS(int engine) const { (void)engine; return 0.0; }

    // Damage (MS-7g): what a hit breaks, each undone by a reset (a reused slot starts whole).
    /// Its engine seized: no thrust, and no relight however it windmills.
    virtual bool seizeEngine(int engine) { (void)engine; return false; }
    /// Every actuator of a control channel stuck where it is (the actuators whose names hold the channel's); how many.
    virtual int failChannel(const std::string& channel) { (void)channel; return 0; }
    /// A part of it gone: its mass (kg) off at a point of its structural frame (m: x aft, y right, z up).
    virtual bool removeMass(double kg, const double structuralM[3]) { (void)kg; (void)structuralM; return false; }

    // A part leaving it (a store released, a section parted, its fuel with it), made consistent: between these two, what
    // changes its mass (removeMass, its tanks' contents, its point masses); then its state point moves to the new CG
    // (JSBSim moves it only on the ground: in the air the airframe jumped by the CG's shift), the rest moving as that
    // point of it did (v + omega x d: what leaves takes the momentum it had there), the part's own inertia off its base
    // and a reaction - an ejector's, a break's spring - on what is left.
    /// Where its CG is now from its model's origin - its empty CG, which a station's or a section's place is given from
    /// - in body axes (m: x forward, y right, z down).
    virtual bool cgFromOriginBodyM(double out[3]) const {
        out[0] = out[1] = out[2] = 0.0;
        return false;
    }
    /// Fuel put into its own tanks (kg; as far as they have room, each its share of the room): what went in.
    virtual double refuel(double kg) {
        (void)kg;
        return 0.0;
    }
    virtual bool beginMassChange() { return false; }
    /// An impulse on it (a blast's, fragments' striking): N s on its CG and N m s about it, body axes.
    virtual bool applyImpulse(const double linearBodyNs[3], const double angularBodyNms[3]) {
        (void)linearBodyNs;
        (void)angularBodyNms;
        return false;
    }
    /// `partInertia`: the part's own about its centre, body axes (kg m^2: Ixx, Iyy, Izz and the plain products
    /// sum(m x y), sum(m x z), sum(m y z); null: none); `impulseBodyNs`: on what is left, body axes (N s; null: none).
    virtual bool endMassChange(const double partInertia[6], const double impulseBodyNs[3]) {
        (void)partInertia;
        (void)impulseBodyNs;
        return false;
    }
};

} // namespace fsim::sim
