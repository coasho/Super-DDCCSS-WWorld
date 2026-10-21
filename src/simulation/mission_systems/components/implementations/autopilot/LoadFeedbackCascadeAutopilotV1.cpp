#include "LoadFeedbackCascadeAutopilotV1.h"
#include <utility>

namespace fsim::ms::components {

LoadFeedbackCascadeAutopilotV1::LoadFeedbackCascadeAutopilotV1(std::string model,weapon::FinControlConfigV1 control)
    : NativePlantBindingV1(std::move(model)),control_(control) {}

weapon::FinCommands LoadFeedbackCascadeAutopilotV1::commands(const weapon::AutopilotInput& input,const weapon::ControlStopsV1& stops) const noexcept {
    return weapon::finCommands(input,control_,stops);
}

} // namespace fsim::ms::components
