#pragma once

// An engagement's fires as NVIDIA Flow's sources (MS-7h, step 3): what a warhead's detonation, a holed tank's spill
// and a leak burning on put into world::FlowSolver, held to what is observed of them. tools/validation/flow_calibrate measures
// the luminous fireball Flow makes from these against the observations (its `check`); world::FlowVolumes draws them.
//
// The calibration (2026-10-09, 0.5 m cells, 60 steps a second; tools/validation/flow_calibrate check, its 12 cases):
// - The combustion: Flow holds a burning cell at its temperature's top (1), so 1 is the flame's: kFlameK, a fuel
//   fireball's radiating soot (CCPS's surface emissive power, some 350 kW/m^2, a black body at 1,576 K); a cell then
//   cools at kCooling a second - the luminous gas's end (its edge 1,200 K), fitted with the detonation's. Combustion
//   makes no expansion and no smoke in Flow: its advection does not dilute what expands, so expansion made by burning
//   compounds without bound (measured: the fireball's size jumps from 0.5 to 7 times the observed between two
//   settings). The products' volume and soot are put in by the sources instead, where the fuel is.
// - A spill (M kg of kerosene at once): CCPS's fireball (Roberts 1981), diameter 5.8 M^(1/3) m (larger as (p0/p)^(1/3)
//   in thinner air), lasting 0.45 M^(1/3) s. Its fuel leaves lit (the warhead's fireball, its fragments) over
//   kSpillReleaseS along the hole's path with kSpillCarry of its speed, a cloud M^(1/3) m round of fuel kSpillFuel
//   M^(1/3) (its burning time); its products' volume, kSpillVolume (100 / M)^0.2 of the fireball's (a fit), made in the
//   cloud over the fireball's first third. Measured 3 - 1000 kg: diameter 0.86 - 1.05 of CCPS's, duration 0.90 - 1.10.
// - A detonation (W kg TNT): its fireball at the photographed size - 5.89 m for 1 kg and 12.05 m for 10 kg in
//   high-speed photography, fired at the ground: hemispheres, which hold the products a sphere 2^(-1/3) as wide holds
//   in free air; larger where the air is thinner by (p0 / p)^(1/3), the products expanding to its pressure (Sachs) -
//   from a core half as wide (wider in thin air: by kDetonationCoreByAir's power of that), hot and sooty, expanding to
//   kDetonationVolume (W / 8)^kDetonationVolumeByMass of its volume (a fit; more in thin air, kDetonationVolumeByAir)
//   over kDetonationGrowSteps steps, burning on for kDetonationBurnS W^(1/3) (an estimate: TNT's afterburning).
//   Measured 1 - 30 kg and 8 kg at 6 km: diameter 0.89 - 1.03, duration 0.97 - 1.11 of the estimate 0.06 W^(1/3) s.
//   (Fitted again 2026-10-11: until then the free-air size was taken a second 2^(-1/3) smaller than photographed -
//   3.71 W^0.31 m for 4.67 - so a warhead's fireball was drawn at some 0.73 of its size.)
// - A leak (R kg/s from a hole at airspeed V): a jet flame, API RP 521's length L = 0.00326 (R 43 MJ/kg)^0.478 m
//   (a flame in still air: a stand-in for one blown back at airspeed); the hole lays lit fuel along its path, burning
//   for what of L its swept tube does not make. Measured 5 - 20 kg/s at 100 - 200 m/s: 0.94 - 1.04 of L.
// - Soot: kSootYield of the fuel (Tewarson's sooty liquid fuel), TNT's kTntSoot, at fresh soot's extinction
//   kSootExtinction (its 7.5 m^2/g absorption, Bond and Bergstrom 2006, over 1 - its albedo 0.2): released with the
//   fuel, the solver's smoke one 1/m of extinction; it then thins at kSmokeDilution a second (an estimate: Flow does
//   not mix it with the air; bounding the solver's near field, a trail some 8 s long - the far trail is not Flow's).
//   Measured before it thins (no dilution): 1.06 of the soot's at sea level, 0.7 - 1.0 at 6 km.

#include "visualization/vfx/flow/FlowSolver.h"

#include <cstdint>
#include <vector>

namespace fsim::world::flowfires {

// the calibration's (the header's comment)
constexpr double kAmbientK = 288.15;
constexpr double kFlameK = 1600.0;            ///< the solver's temperature 1: a fuel fireball's radiating soot (CCPS's 350 kW/m^2)
constexpr double kCooling = 8.0;              ///< 1/s
constexpr double kSpillReleaseS = 0.1;
constexpr double kSpillCarry = 0.3;           ///< the spray's share of the hole's speed as it leaves (an estimate)
constexpr double kSpillFuel = 0.4;            ///< per kg^(1/3)
constexpr double kSpillVolume = 0.85;
constexpr double kDetonationVolume = 2.6, kDetonationBurnS = 0.055, kDetonationCore = 0.5;
/// the fit's powers: the expansion's of W / 8 kg and of the thin air's (p0 / p)^(1/3), the core's of that; and the
/// solver's steps the expansion is given over
constexpr double kDetonationVolumeByMass = 0.12, kDetonationVolumeByAir = 0.8, kDetonationCoreByAir = 0.5;
constexpr double kDetonationGrowSteps = 3.0;
constexpr double kLeakRadiusM = 1.5;          ///< the hole's spray (an estimate: a few cells)
constexpr double kSootYield = 0.04, kTntSoot = 0.05;
constexpr double kSootExtinction = 9375.0;    ///< m^2 per kg of soot
constexpr double kSmokeDilution = 0.4;        ///< the smoke's share lost a second (an estimate: a plume's dilution; the
                                              ///< solver's near field, a trail some 8 s long)
constexpr double kVelocityDamping = 0.2;      ///< the gas's motion's share lost a second (an estimate)
constexpr double kProductsM3PerKg = 102.0;    ///< a spill's fireball's volume per kg at sea level (CCPS's diameter)

/// The air a fire burns in, and the wind that carries its smoke (from, deg; m/s).
struct Air {
    double seaLevelK = 288.15, seaLevelPa = 101325.0, windFromDeg = 0.0, windMs = 0.0;
};
/// The pressure (Pa) at a height in that air (the standard atmosphere from the sea level's).
double pressureAt(const Air& air, double altitudeM);

// what is observed
double ccpsDiameter(double fuelKg, double pressurePa);
double ccpsDuration(double fuelKg);
double detonationDiameter(double tntKg, double pressurePa); ///< in free air: 5.89 W^0.31 m fired at the ground, 2^(-1/3) of it (4.67 W^0.31)
double detonationDuration(double tntKg);                    ///< (an estimate)
double jetFlameLength(double rateKgS);                      ///< API RP 521

/// The layer's combustion (the solver's settings: cell 0.5 m, 60 steps a second).
FlowSolver::Combustion combustion();

// The sources at an age (s; from when it happened), for a step of dt, each its luid (its own, step to step; a source
// uses luid and luid + 1). Positions and velocities in the solver's frame (m, m/s; the air's). Each false once done.
bool detonation(double tntKg, const double at[3], const double drift[3], double pressurePa, double age, double dt,
                std::uint64_t luid, std::vector<FlowSolver::Sphere>& out);
bool spill(double fuelKg, const double at[3], const double path[3], double pressurePa, double age, double dt, std::uint64_t luid,
           std::vector<FlowSolver::Sphere>& out);
/// An aircraft's wake where it lays it now: its vortex pair - each Gamma = W / (rho V b0) round a Lamb-Oseen core,
/// b0 = pi/4 of its span apart (an elliptic wing's), the right one turning down inboard - given to the cells it passes
/// over the call, the solver carrying it on; what rolls a burning aircraft's smoke into its vortices. `at` its middle,
/// `velocity` through the air, `right` and `up` its body's (the solver's frame), `dt` s since the last call.
void wake(double massKg, double spanM, const double at[3], const double velocity[3], const double right[3], const double up[3],
          double pressurePa, double airK, double dt, std::uint64_t luid, std::vector<FlowSolver::Sphere>& out);
/// The pair's circulation (m^2/s) and core radius (m) for this aircraft at this airspeed and air density.
double wakeCirculation(double massKg, double spanM, double speedMs, double densityKgM3);
constexpr double kWakeCore = 0.05;            ///< the vortex core's radius, of the span (an estimate: 3 - 5 % measured)
constexpr int kWakeGrid = 3;                  ///< the points a side the pair's flow is given at
constexpr double kKeepSpeed = 3.0;            ///< m/s: a block with no smoke freed once slower (an estimate; Flow's 1)
constexpr double kWakeTurbulence = 0.05;      ///< a near wake's turbulence, rms of the airspeed (an estimate)

/// A leak at its hole now (`at`), the hole moving at `velocity` through the air, `dt` s since the last call.
void leak(double rateKgS, const double at[3], const double velocity[3], double pressurePa, double dt, std::uint64_t luid,
          std::vector<FlowSolver::Sphere>& out);

} // namespace fsim::world::flowfires
