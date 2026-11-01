#include "ActuatorAwareCascadeAutopilotV1.h"
#include <utility>
namespace fsim::ms::components {
ActuatorAwareCascadeAutopilotV1::ActuatorAwareCascadeAutopilotV1(std::string model,weapon::FinControlConfigV1 control)
    : NativePlantBindingV1(std::move(model)),control_(control) {}
weapon::FinCommands ActuatorAwareCascadeAutopilotV1::commands(const weapon::AutopilotInput& in,const weapon::ControlStopsV1& stops) const noexcept {
    return weapon::actuatorAwareCommands(in,control_,stops);
}
}
