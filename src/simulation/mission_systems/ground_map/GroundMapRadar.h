#pragma once

// SIM-M07 (MS-11): Real-Beam Ground-Mapping & Terrain-Avoidance Radar (`an-apq-166`, `h6k-radar`).
// Preserves `CapabilityType::Status`-only A-GRA contract while publishing terrain clearance
// contacts (`ContactKind::Terrain`) and X-band emissions when operating.

#include "simulation/mission_systems/Model.h"

namespace fsim::ms::ground_map {

/// Construct a real-beam ground-map / terrain-avoidance radar model from its card.
std::unique_ptr<Model> makeGroundMapRadar(const Card& card, std::uint16_t count);

} // namespace fsim::ms::ground_map
