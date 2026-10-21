#pragma once

#include "simulation/mission_systems/components/interfaces/AutopilotProviderV1.h"
#include "simulation/mission_systems/components/support/NativePlantBindingV1.h"

namespace fsim::ms::components {

class LoadFeedbackCascadeAutopilotV1 final : public NativePlantBindingV1<AutopilotProviderV1> {
public:
    LoadFeedbackCascadeAutopilotV1(std::string model,weapon::FinControlConfigV1 control);
    weapon::FinCommands commands(const weapon::AutopilotInput& input,const weapon::ControlStopsV1& stops) const noexcept override;
private:
    weapon::FinControlConfigV1 control_;
};

} // namespace fsim::ms::components
