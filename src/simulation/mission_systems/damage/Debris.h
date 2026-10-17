#pragma once

// What the damage chain needs of the Earth and its air (MS-7g): a point's geodetic place, the standard atmosphere's
// density. (A piece that leaves a hit aircraft flies its own JSBSim model: hangar's pieces,
// simulation/runtime/world/WorldDamage.cpp.)

#include "simulation/mission_systems/damage/Damage.h"

namespace fsim::ms::damage {

/// WGS-84: a point's geodetic latitude, longitude (rad) and height (m) (Bowring's).
void geodetic(const V3& ecef, double& latitudeRad, double& longitudeRad, double& heightM) noexcept;
/// The standard atmosphere's density at a height (kg/m^3; ISA's troposphere, isothermal above 11 km).
double standardDensity(double heightM) noexcept;

} // namespace fsim::ms::damage
