#pragma once

#include "simulation/mission_systems/components/interfaces/AerodynamicsProviderV1.h"
#include "simulation/mission_systems/components/support/NativePlantBindingV1.h"

namespace fsim::ms::components {

class JsbsimGeneratedStoreAerodynamicsV1 final : public NativePlantBindingV1<AerodynamicsProviderV1> {
public:
    JsbsimGeneratedStoreAerodynamicsV1(std::string model,std::string source);
    std::string_view coefficientSource() const noexcept override;
private:
    std::string source_;
};

} // namespace fsim::ms::components
