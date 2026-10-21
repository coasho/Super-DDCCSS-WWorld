#pragma once

#include "simulation/mission_systems/components/interfaces/WarheadAdapter.h"

#include "Damage.h"
#include "simulation/mission_systems/components/Profile.h"

#include <memory>
#include <string_view>

namespace fsim::ms::damage {

/// Available providers only. Invalid, unavailable and not-applicable
/// declarations return null with a reason; they never select another model.
std::unique_ptr<WarheadAdapter> makeWarheadAdapter(
    const StoreCard&, const components::DescriptorV1&, std::string& error);

} // namespace fsim::ms::damage
