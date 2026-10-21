#include "BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/implementations/autopilot/LoadFeedbackCascadeAutopilotV1.h"

namespace fsim::ms::components {

void registerNativeAutopilotV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.autopilot.registerFactory("fsim.control.load_feedback_cascade","fsim.control.load_feedback_cascade",
        [](const DescriptorV1&,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<AutopilotProviderV1> {
            e.clear();return std::make_unique<LoadFeedbackCascadeAutopilotV1>(ctx.card.model,ctx.spec.control);
        },false,error);
}

} // namespace fsim::ms::components
