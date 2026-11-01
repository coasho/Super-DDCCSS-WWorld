#pragma once
#include "simulation/mission_systems/components/interfaces/PropulsionProviderV1.h"
#include "simulation/mission_systems/components/support/NativePlantBindingV1.h"

namespace fsim::ms::components {
class JsbsimDuctedRocketPropulsionV1 final : public NativePlantBindingV1<PropulsionProviderV1> {
public:
    explicit JsbsimDuctedRocketPropulsionV1(std::string model);
    bool acceptsEngineCount(unsigned count) const noexcept override;
    bool burning(double nativeThrustN) const noexcept override;
};
}
