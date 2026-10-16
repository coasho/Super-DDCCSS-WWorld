#pragma once

// Envelope protection (docs/control-architecture.md, section 11): the demand
// the control system passes on, limited to the aircraft's envelope, and the
// state checked against it. It limits what is demanded; it never promises
// where the aircraft goes. A gust, inertia, a saturated surface or a law the
// profile describes imperfectly can still take it past a limit: that is
// reported, not prevented.

#include "simulation/flight_control/Runtime.h"
#include "fsim/VehicleProfile.h"
#include "fsim/VehicleState.h"

#include <array>
#include <cstdint>
#include <limits>

namespace fsim::control {

using LimitMask = std::uint16_t;
constexpr LimitMask limitBit(Limit l) noexcept { return static_cast<LimitMask>(1u << static_cast<unsigned>(l)); }

/// The axis whose control flies against a limit (6.3): load factor, alpha
/// and pitch the pitch; bank and roll rate the roll; airspeed and Mach the thrust.
inline Axis limitAxis(Limit l) noexcept {
    constexpr Axis kAxes[kLimitCount] = {Axis::Pitch, Axis::Pitch, Axis::Pitch, Axis::Roll, Axis::Pitch,
                                         Axis::Pitch, Axis::Roll,  Axis::Thrust, Axis::Thrust, Axis::Thrust};
    return kAxes[static_cast<std::size_t>(l)];
}

/// A turn's bank and its least airspeed (docs/vehicle-interface.md, 7.3): no steeper than leaves the wing a margin over
/// the lift its least airspeed has - that airspeed's load factor over kTurnMargin - and kTurnBankFloorRad at least, so
/// that it still turns. Slower than 1.16 times its least it is the floor's 15 deg; 40 deg at 1.3 times; 60 deg from 1.6.
inline constexpr double kTurnMargin = 1.3;
inline constexpr double kTurnBankFloorRad = 0.2617993877991494; // 15 deg
/// That bank at calibrated airspeed `casMs`, for a least airspeed `casMinMs`; infinite where none is known.
double turnBankRadFor(double casMs, double casMinMs) noexcept;

/// The speed floor (docs/vehicle-interface.md, 7.3): what a wing flies instead of an airspeed it cannot hold its height
/// at. Slow - within kFloorReach of its least airspeed - asked level or to climb, and sinking for kFloorDelayS, its
/// airspeed setpoint is raised from the speed it has, kFloorRateMs2 a second, until it sinks no more; and held there
/// until a command asks at least as much. Clean only: with its flaps or its gear out it is being landed. A floor
/// within kFloorSlightMs of the airspeed asked - a turn's sink, a gust's: the airspeed asked is one it holds - is let go
/// once it has flown level at it, wings level, for kFloorSteadyS.
struct SpeedFloor {
    double casMs = 0.0;    ///< the floor, calibrated; 0: none
    double sinkingS = 0.0; ///< how long it has sunk when asked not to
    bool active = false;   ///< this update, it raised the airspeed asked
    bool seen = false;     ///< this update, a velocity setpoint was limited (else the floor is forgotten)
    double steadyS = 0.0;  ///< how long a slight floor has been flown level
};
inline constexpr double kFloorSlightMs = 2.0;  ///< a floor no more than this above the airspeed asked is slight
inline constexpr double kFloorSteadyS = 15.0;  ///< and let go after this long level at it
inline constexpr double kFloorLevelBankRad = 0.0873; ///< (wings within 5 deg of level)
inline constexpr double kFloorReach = 1.3;   ///< times the least airspeed: slower than this, a sink is the speed's
inline constexpr double kFloorSinkMs = 1.0;  ///< sinking faster than this, asked level
inline constexpr double kFloorDelayS = 3.0;  ///< for this long (a level-off's own lag is shorter)
inline constexpr double kFloorRateMs2 = 1.0; ///< the floor's rise
inline constexpr double kFloorMost = 1.6;    ///< times the least airspeed: the floor goes no higher

/// What the limits need from the state, worked out once per update.
struct LimitState {
    const sim::VehicleState* s = nullptr;
    double tasPerCas = 1.0;  ///< a calibrated airspeed limit, as true airspeed here
    double tasPerMach = 0.0; ///< the speed of sound here (0: not known, below Mach 0.05)
    double bankCos = 1.0;    ///< cos(bank), floored at 0.1
    double bankCapRad = std::numeric_limits<double>::infinity(); ///< the steepest bank its airspeed leaves a turn (turnBankRadFor)
    SpeedFloor* floor = nullptr; ///< the vehicle's speed floor, kept across updates; null: none kept (a setpoint limited on its own)
    double dt = 0.0;             ///< the control update's, for the floor
    bool clean = true;           ///< its flaps in and its gear up: the floor is armed
};
/// Only what `e` asks for is worked out (a division is a division).
LimitState limitState(const sim::VehicleState& s, const EnvelopeLimits& e, const Protection& p) noexcept;

/// The protection a profile gives: its envelope's limits (the flaps
/// configuration completed from the clean one), what the aircraft's own law
/// enforces, the plant's numbers the elevator limiter scales by. Limit if the
/// profile has an envelope section, else Off (D3).
Protection protectionFor(const VehicleProfile& profile, std::uint32_t features = kFeatureWingborne) noexcept;

/// The limits in force: the flaps configuration's with the flaps commanded
/// beyond the threshold, the gear's speed with the gear down (then written into `scratch`).
const EnvelopeLimits& activeLimits(const Protection& p, double flapsCommand, double gearCommand, EnvelopeLimits& scratch) noexcept;

/// A setpoint at its level (above the actuators) limited as the state is
/// now (11.3): each field to its limit, the pitch attitude and the load
/// factor also to what puts the wing at alpha_max here (unless the
/// aircraft's law enforces alpha). kHold fields are left alone. Returns the
/// limits that changed it.
LimitMask limitSetpoint(Command& c, const EnvelopeLimits& e, const Protection& p, const LimitState& here) noexcept;
inline LimitMask limitSetpoint(Command& c, const EnvelopeLimits& e, const Protection& p, const sim::VehicleState& s) noexcept {
    return limitSetpoint(c, e, p, limitState(s, e, p));
}

/// The elevator of a surface-controlled aircraft, moved nose-down as the
/// load factor or alpha nears its maximum and nose-up as the load factor
/// nears its minimum - never towards a limit. Returns the limits that moved it.
LimitMask limitElevator(double& elevator, const EnvelopeLimits& e, const Protection& p, const sim::VehicleState& s) noexcept;

/// The state against the limits, in flight: each limit it is beyond, and by
/// how much in `excess` (in the limit's unit; the others' entries are left alone).
LimitMask exceeded(const EnvelopeLimits& e, const sim::VehicleState& s, std::array<double, kLimitCount>& excess) noexcept;

} // namespace fsim::control
