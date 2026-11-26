#pragma once

// SIM-M07 (MS-11): Surface Moving-Target Indication (SMTI) Radar.
// Aligned with Skolnik / Richards endo-clutter Minimum Detectable Velocity (MDV).
// Retains the no-ground-entities boundary: reports zero surface contacts when no
// ground/low-AGL movers exist in the scene.

#include "simulation/mission_systems/Model.h"

namespace fsim::ms::smti {

/// Endo-clutter minimum detectable radial velocity: MDV = (lambda / 2) * clutterSpreadHz.
double minimumDetectableVelocityMs(double frequencyHz, double clutterSpreadHz) noexcept;

/// Construct an SMTI payload model from its card.
std::unique_ptr<Model> makeSmtiRadar(const Card& card, std::uint16_t count);

} // namespace fsim::ms::smti
