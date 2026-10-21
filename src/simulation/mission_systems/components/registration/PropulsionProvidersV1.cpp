#include "BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/implementations/propulsion/JsbsimSolidRocketPropulsionV1.h"

namespace fsim::ms::components {

void registerNativePropulsionV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.propulsion.registerFactory("fsim.propulsion.jsbsim_generated_solid_rocket","fsim.propulsion.jsbsim_generated_solid_rocket",
        [](const DescriptorV1&,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<PropulsionProviderV1> {
            e.clear();return std::make_unique<JsbsimSolidRocketPropulsionV1>(ctx.card.model);
        },false,error);
}

} // namespace fsim::ms::components
