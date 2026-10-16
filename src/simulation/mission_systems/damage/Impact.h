#pragma once

// A vehicle striking the ground (MS-7g's damage chain, its last link): what the strike does to it, from how it came in.
// The same aircraft at the same speed leaves a crater under a column of fire when it dives in, and a burning trail of
// wreckage a kilometre long when it flies into level ground: what decides is the split of its velocity across and
// along the surface, and the forces that stopping it takes.
//
// - Across the surface it is stopped within what it has to crush - its body's depth and the ground's give: its mean
//   deceleration v_n^2 / (2 d), for 2 d / v_n. That is the load its structure carries (session::World hands it to the
//   damage model as an accelerometer's reading: a section whose root cannot hold its own weight that many times over
//   breaks off, as one does under a flight load).
// - Along the surface the ground can only drag it: Coulomb's impulse, mu (1 + e) m v_n against its motion - all of it
//   only when it comes in steeply. What leaves the strike has that tangential velocity and e v_n of rebound: a dive's
//   wreckage stays where it struck, a shallow strike's goes on along its track.
// - Its fuel: tanks burst once the strike is beyond what a crashworthy fuel system is built for (MIL-STD-1290's design
//   vertical impact, 12.8 m/s), all of them well beyond it; of what is spilt a share is thrown out as mist and burns
//   at once - the fireball, moving with the wreckage - and the rest burns on where it lies.
//
// Estimates, each said where it is: the crush depth from the mass alone, the ground's friction and restitution, the
// mist's share. None is fitted to a crash; they are the measured ranges' middles.

#include "simulation/mission_systems/damage/Damage.h"

namespace fsim::ms::damage {

constexpr double kBulkDensityKgM3 = 120.0; ///< an aircraft's mass over its body's volume (an estimate: 80 - 160)
constexpr double kCrushShare = 0.25;       ///< of its body's size (m / rho)^(1/3): about half its depth
constexpr double kGroundGiveM = 0.3;       ///< what soil gives under it (an estimate; rock none, water more)
constexpr double kGroundFriction = 0.6;    ///< an airframe ploughing soil (an estimate: sliding metal on soil 0.4 - 0.8)
constexpr double kRestitution = 0.1;       ///< a crumpling airframe's (an estimate: 0.05 - 0.2)
constexpr double kTankSafeMs = 12.8;       ///< MIL-STD-1290A's vertical design impact (42 ft/s): tanks hold to here
constexpr double kTankBurstMs = 45.0;      ///< every tank burst (an estimate: some 3.5 times the design impact)
constexpr double kSlideShare = 0.15;       ///< what a metre a second along the ground counts against one into it (an
                                           ///< estimate: a wheels-up landing at 70 m/s holds its fuel, a strike at 200 does not)
constexpr double kMistMost = 0.4;          ///< of the spilt fuel, at most, that burns at once as the fireball (an estimate)
constexpr double kMistFullMs = 100.0;      ///< the severity at which that much does
constexpr double kPoolBurnS = 90.0;        ///< what is left burns on this long at least (an estimate: a pool fire's minutes)

/// How it came in: its velocity as it struck and the ground's unit normal (any one frame), its mass and its fuel.
struct GroundStrike {
    V3 velocity, normal;
    double massKg = 0.0, fuelKg = 0.0;
};

struct StrikeOutcome {
    double speedMs = 0.0, normalMs = 0.0, tangentMs = 0.0;
    double angleRad = 0.0;        ///< its path below the surface: 0 along it, pi/2 straight in
    double energyJ = 0.0, normalEnergyJ = 0.0;
    double crushM = 0.0, stopS = 0.0;
    double decelMs2 = 0.0;        ///< across the surface, the mean over the crush
    V3 specificForce;             ///< what an accelerometer in it reads through the crush (the strike's frame): the ground's
                                  ///< push up the normal and its drag against the motion
    V3 after;                     ///< the velocity of what leaves the strike
    double severityMs = 0.0;      ///< its speed into the ground and its slide's share (what its tanks are held to)
    double spiltShare = 0.0;      ///< of its fuel: the tanks that burst
    double fireballKg = 0.0;      ///< burning at once, as mist
    double poolKg = 0.0, poolKgS = 0.0; ///< what burns on where it lies, and how fast
};

/// What the strike does. A velocity leaving the ground (a touch, a bounce) is no strike: all zero but its speeds.
StrikeOutcome groundStrike(const GroundStrike& in) noexcept;

} // namespace fsim::ms::damage
