#pragma once

#include "ProviderTableV1.h"
#include "PlantProvidersV1.h"
#include "interfaces/SeekerProviderV1.h"
#include "interfaces/GuidanceProviderV1.h"
#include "interfaces/FuzeAdapter.h"
#include "interfaces/WarheadAdapter.h"

namespace fsim::ms::components {

class ProviderRegistryV1 {
public:
    ProviderTableV1<SeekerProviderV1> seeker{"seeker"};
    ProviderTableV1<GuidanceProviderV1> guidance{"guidance"};
    ProviderTableV1<weapon::FuzeAdapter> fuze{"fuze"};
    ProviderTableV1<damage::WarheadAdapter> warhead{"warhead"};
    ProviderTableV1<AutopilotProviderV1> autopilot{"autopilot"};
    ProviderTableV1<AllocatorProviderV1> allocator{"allocator"};
    ProviderTableV1<ActuatorProviderV1> actuator{"actuator"};
    ProviderTableV1<PropulsionProviderV1> propulsion{"propulsion"};
    ProviderTableV1<AerodynamicsProviderV1> aerodynamics{"aerodynamics"};
    bool validationProfile(const ProfileV1&, ProfileV1&, std::string& error) const;
    bool createPlant(const ProfileV1&, const ProviderContextV1&, const weapon::PlantAdapterV1&,
                     PlantProvidersV1&, std::string& error) const;
};

} // namespace fsim::ms::components
