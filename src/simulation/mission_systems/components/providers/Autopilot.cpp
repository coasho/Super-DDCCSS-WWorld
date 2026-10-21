#include "NativePlantBinding.h"
namespace fsim::ms::components {
namespace {
class LoadFeedbackCascadeAutopilotV1 final : public NativePlantBindingV1<AutopilotProviderV1> {
public:
    LoadFeedbackCascadeAutopilotV1(std::string model,weapon::FinControlConfigV1 control)
        : NativePlantBindingV1(std::move(model)),control_(control) {}
    weapon::FinCommands commands(const weapon::AutopilotInput& input,const weapon::ControlStopsV1& stops) const noexcept override {
        return weapon::finCommands(input,control_,stops);
    }
private:
    weapon::FinControlConfigV1 control_;
};
}
void registerNativeAutopilotV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.autopilot.registerFactory("fsim.control.load_feedback_cascade","fsim.control.load_feedback_cascade",
        [](const DescriptorV1&,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<AutopilotProviderV1> {
            e.clear();return std::make_unique<LoadFeedbackCascadeAutopilotV1>(ctx.card.model,ctx.spec.control);
        },false,error);
}
}
