#include "BuiltInProvidersV1.h"
#include "simulation/mission_systems/weapon/FuzeAdapter.h"
namespace fsim::ms::components {
void registerNativeFuzeV1(ProviderRegistryV1& registry) {
    std::string error;
    for(const auto* id:{"native.fuze.legacy.v1","native.fuze.swept_path.v1","native.fuze.turning_cpa.v1"})
        registry.fuze.registerFactory(id,id,[](const DescriptorV1& d,const ProviderContextV1& ctx,std::string& e) {
            return weapon::makeFuzeAdapter(d,ctx.spec,e);
        },false,error);
}
}
