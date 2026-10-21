#pragma once

#include "interfaces/AutopilotProviderV1.h"
#include "interfaces/AllocatorProviderV1.h"
#include "interfaces/ActuatorProviderV1.h"
#include "interfaces/PropulsionProviderV1.h"
#include "interfaces/AerodynamicsProviderV1.h"
#include <memory>

namespace fsim::ms::components {

struct PlantProvidersV1 {
    std::unique_ptr<AutopilotProviderV1> autopilot;
    std::unique_ptr<AllocatorProviderV1> allocator;
    std::unique_ptr<ActuatorProviderV1> actuator;
    std::unique_ptr<PropulsionProviderV1> propulsion;
    std::unique_ptr<AerodynamicsProviderV1> aerodynamics;
    bool available(const weapon::PlantAdapterV1&, const weapon::NativePlantIdentityV1&) const noexcept;
    weapon::FinCommands commands(const weapon::PlantAdapterV1&, const weapon::AutopilotInput&,
        const weapon::ControlStopsV1&, const weapon::NativePlantIdentityV1&) const noexcept;
};

} // namespace fsim::ms::components
