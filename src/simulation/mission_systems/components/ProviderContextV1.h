#pragma once

#include "Profile.h"
#include "simulation/mission_systems/weapon/Guidance.h"

namespace fsim::ms::components {

struct ProviderContextV1 {
    const StoreCard& card;
    const ProfileV1& profile;
    weapon::Spec spec;
    std::optional<weapon::Observation> initialEstimate;
};

} // namespace fsim::ms::components
