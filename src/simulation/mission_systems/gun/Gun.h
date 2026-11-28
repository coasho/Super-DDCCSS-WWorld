#pragma once

// SIM-M10: Aircraft Gun Model & Exterior Ballistics Helpers (McCoy / NATO STANAG 4355).
//
// Covers all 11 maintained Family::Gun cards (m61a1, m61a2, gau-22a, gau-8a, bk-27,
// defa-554, giat-30m791, gsh-23, gsh-30-1, gsh-30-2, m60d) with published calibre,
// cyclic rate of fire, muzzle velocity, projectile mass, ammunition mass conservation
// (B-33), HEI filling, and quadratic atmospheric drag retardation.

#include "simulation/mission_systems/Cards.h"
#include "simulation/mission_systems/Model.h"

#include <cstdint>
#include <memory>

namespace fsim::ms::gun {

struct Spec {
    double calibreM = 0.020;           ///< projectile diameter (m)
    double rateOfFireRpm = 6000.0;     ///< combined cyclic rate across installed barrels/mounts (rpm)
    double muzzleMs = 1050.0;          ///< muzzle velocity (m/s)
    double projectileKg = 0.101;       ///< projectile mass leaving barrel (kg)
    double shedMassPerRoundKg = 0.155; ///< total mass leaving aircraft per round fired (B-33)
    double explosiveKg = 0.011;        ///< HEI explosive fill per round (kg; 0 for solid ball)
    double dispersionRad = 0.002;      ///< 1-sigma angular dispersion per axis (rad)
    double dragCd = 0.30;              ///< supersonic-averaged G7 projectile drag coefficient
    double elevationRad = 0.0;         ///< fixed gun harmonisation elevation above body x-axis (rad)
    int capacity = 500;                ///< total ammunition capacity across mounts (rounds)
    double maxFlightTimeS = 6.0;       ///< pyrotechnic self-destruct fuze / max ballistic lifetime (s)
};

/// Published BRL / McCoy G7 boattail projectile drag coefficient Cd(M) vs Mach number
/// (R. L. McCoy, "Modern Exterior Ballistics", 1999; NATO STANAG 4355).
double g7DragCoefficient(double mach) noexcept;

/// Sea-level quadratic drag retardation factor k = (rho0 * Cd * A) / (2 * m) in 1/m,
/// so dv/dt = -k * (rho / rho0) * |v| * v (McCoy, Modern Exterior Ballistics).
double dragRetardationPerM(double calibreM, double projectileKg, double dragCd = 0.30) noexcept;

/// Resolve published/derived gun parameters for any maintained Family::Gun card.
Spec specOf(const Card& card, std::uint16_t count = 1) noexcept;

/// Create a gun payload model for a Family::Gun card.
std::unique_ptr<Model> makeGun(const Card& card, std::uint16_t count = 1);

} // namespace fsim::ms::gun
