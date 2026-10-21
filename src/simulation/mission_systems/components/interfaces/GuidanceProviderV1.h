#pragma once

#include "simulation/mission_systems/weapon/Guidance.h"

namespace fsim::ms::components {

class GuidanceProviderV1 {
public:
    virtual ~GuidanceProviderV1() = default;
    virtual weapon::Spec spec() const noexcept = 0;
    virtual weapon::Output update(const weapon::Input&) = 0;
};

} // namespace fsim::ms::components
