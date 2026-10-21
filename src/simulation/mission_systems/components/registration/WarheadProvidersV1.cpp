#include "BuiltInProvidersV1.h"
#include "simulation/mission_systems/damage/WarheadAdapter.h"
namespace fsim::ms::components {
void registerNativeWarheadV1(ProviderRegistryV1& registry) {
    std::string error;
    for(const auto* id:{"native.warhead.legacy.v1","native.warhead.zoned_case.v1","native.warhead.side_case.v1"})
        registry.warhead.registerFactory(id,id,[](const DescriptorV1& d,const ProviderContextV1& ctx,std::string& e) {
            return damage::makeWarheadAdapter(ctx.card,d,e);
        },false,error);
}
}
