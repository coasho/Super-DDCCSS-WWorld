#pragma once

#include "simulation/mission_systems/components/interfaces/GuidanceProviderV1.h"

namespace fsim::ms::components {

class VelocityNormalPnGuidanceV1 final : public GuidanceProviderV1 {
public:
    VelocityNormalPnGuidanceV1(weapon::Spec spec,std::optional<weapon::Observation> estimate);
    weapon::Spec spec() const noexcept override;
    weapon::Output update(const weapon::Input& input) override;
private:
    weapon::Guidance native_;
};

} // namespace fsim::ms::components
