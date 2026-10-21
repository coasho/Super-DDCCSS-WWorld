#pragma once

#include "simulation/mission_systems/components/interfaces/FuzeAdapter.h"

#include "ContactFuze.h"
#include "FuzeDelay.h"
#include "MotionHistory.h"
#include "simulation/mission_systems/components/Profile.h"

#include <memory>
#include <string_view>

namespace fsim::ms::weapon {

/// Copies the native specification, configuring only its fuze fields from the
/// explicit descriptor. Refusal never modifies the caller's specification.
std::unique_ptr<FuzeAdapter> makeFuzeAdapter(
    const components::DescriptorV1&, const Spec&, std::string& error);

} // namespace fsim::ms::weapon
