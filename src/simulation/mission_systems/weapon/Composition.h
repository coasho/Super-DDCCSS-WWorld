#pragma once

#include "Guidance.h"
#include "simulation/mission_systems/components/Profile.h"

namespace fsim::ms::components { class ProviderRegistryV1; }
namespace fsim::ms::weapon {

/// Atomic armed-guided composition. Version zero preserves the exact baseline;
/// explicit profiles must agree at shared mode/capability/lifetime/body seams.
bool configureCompositionV1(const StoreCard& card, const components::ProfileV1& profile,
                            const Spec& baseline, Spec& output, std::string& error);
bool configureCompositionV1(const StoreCard& card, const components::ProfileV1& profile,
                            const Spec& baseline, Spec& output, std::string& error,
                            const components::ProviderRegistryV1& providers);

} // namespace fsim::ms::weapon
