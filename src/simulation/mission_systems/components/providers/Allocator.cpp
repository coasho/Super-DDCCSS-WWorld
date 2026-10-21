#include "NativePlantBinding.h"
#include <cmath>
namespace fsim::ms::components {
namespace {
class JsbsimAuthoredFinMixerV1 final : public NativePlantBindingV1<AllocatorProviderV1> {
public:
    explicit JsbsimAuthoredFinMixerV1(std::string model) : NativePlantBindingV1(std::move(model)) {}
    weapon::FinCommands allocate(weapon::FinCommands input,const weapon::ControlStopsV1&) const noexcept override {
        // JSBSim's authored mixer consumes these virtual channels. Do not
        // duplicate mixing or infer a replacement physical-fin allocator here.
        if(!input.available || !std::isfinite(input.aileron) || !std::isfinite(input.elevator) || !std::isfinite(input.rudder) ||
            std::abs(input.aileron)>1 || std::abs(input.elevator)>1 || std::abs(input.rudder)>1)return {};
        return input;
    }
};
}
void registerNativeAllocatorV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.allocator.registerFactory("fsim.allocator.jsbsim_authored_fin_mixer","fsim.allocator.jsbsim_authored_fin_mixer",
        [](const DescriptorV1&,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<AllocatorProviderV1> {
            e.clear();return std::make_unique<JsbsimAuthoredFinMixerV1>(ctx.card.model);
        },false,error);
}
}
