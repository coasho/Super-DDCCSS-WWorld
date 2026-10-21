#pragma once

#include "simulation/mission_systems/components/interfaces/AllocatorProviderV1.h"
#include "simulation/mission_systems/components/support/NativePlantBindingV1.h"

namespace fsim::ms::components {

class JsbsimAuthoredFinMixerV1 final : public NativePlantBindingV1<AllocatorProviderV1> {
public:
    explicit JsbsimAuthoredFinMixerV1(std::string model);
    weapon::FinCommands allocate(weapon::FinCommands input,const weapon::ControlStopsV1&) const noexcept override;
};

} // namespace fsim::ms::components
