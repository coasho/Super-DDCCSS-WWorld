#include "BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/implementations/aerodynamics/JsbsimGeneratedStoreAerodynamicsV1.h"

namespace fsim::ms::components {

void registerNativeAerodynamicsV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.aerodynamics.registerFactory("fsim.aerodynamics.jsbsim_generated_store","fsim.aerodynamics.jsbsim_generated_store",
        [](const DescriptorV1& d,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<AerodynamicsProviderV1> {
            const auto source=d.bindings.find("source");
            if(source==d.bindings.end()){e="native aerodynamic source missing";return {};}
            e.clear();return std::make_unique<JsbsimGeneratedStoreAerodynamicsV1>(ctx.card.model,source->second);
        },false,error);
}

} // namespace fsim::ms::components
