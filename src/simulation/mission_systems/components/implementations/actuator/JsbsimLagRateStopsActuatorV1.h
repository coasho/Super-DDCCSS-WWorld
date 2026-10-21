#pragma once

#include "simulation/mission_systems/components/interfaces/ActuatorProviderV1.h"
#include "simulation/mission_systems/components/support/NativePlantBindingV1.h"

namespace fsim::ms::components {

class JsbsimLagRateStopsActuatorV1 final : public NativePlantBindingV1<ActuatorProviderV1> {
public:
    JsbsimLagRateStopsActuatorV1(std::string model,double lag,double rate);
    bool accepts(const weapon::AutopilotInput& input,const weapon::ControlStopsV1& stops) const noexcept override;
private:
    double lag_,rate_;
};

} // namespace fsim::ms::components
