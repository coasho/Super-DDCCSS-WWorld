#include "BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/implementations/allocator/JsbsimAuthoredFinMixerV1.h"

namespace fsim::ms::components {

void registerNativeAllocatorV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.allocator.registerFactory("fsim.allocator.jsbsim_authored_fin_mixer","fsim.allocator.jsbsim_authored_fin_mixer",
        [](const DescriptorV1&,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<AllocatorProviderV1> {
            e.clear();return std::make_unique<JsbsimAuthoredFinMixerV1>(ctx.card.model);
        },false,error);
}

} // namespace fsim::ms::components
