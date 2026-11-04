#include "BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/implementations/autopilot/LoadFeedbackCascadeAutopilotV1.h"
#include "simulation/mission_systems/components/implementations/autopilot/ActuatorAwareCascadeAutopilotV1.h"
#include "simulation/mission_systems/components/implementations/autopilot/OperatingPointAutopilotV1.h"

namespace fsim::ms::components {

void registerNativeAutopilotV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.autopilot.registerFactory("fsim.control.operating_point_bounded.v1","fsim.control.operating_point_bounded.v1",
        [](const DescriptorV1&,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<AutopilotProviderV1> {
            e.clear();return std::make_unique<OperatingPointAutopilotV1>(ctx.card.model,ctx.spec.control);
        },false,error);
    registry.autopilot.registerFactory("fsim.control.load_feedback_cascade","fsim.control.load_feedback_cascade",
        [](const DescriptorV1&,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<AutopilotProviderV1> {
            e.clear();return std::make_unique<LoadFeedbackCascadeAutopilotV1>(ctx.card.model,ctx.spec.control);
        },false,error);
    registry.autopilot.registerFactory("fsim.control.actuator_aware_cascade.v1","fsim.control.actuator_aware_cascade.v1",
        [](const DescriptorV1&,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<AutopilotProviderV1> {
            e.clear();return std::make_unique<ActuatorAwareCascadeAutopilotV1>(ctx.card.model,ctx.spec.control);
        },false,error);
}

} // namespace fsim::ms::components
