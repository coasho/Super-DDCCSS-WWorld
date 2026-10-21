#include "BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/implementations/guidance/VelocityNormalPnGuidanceV1.h"

namespace fsim::ms::components {

std::unique_ptr<GuidanceProviderV1> nativeGuidanceProviderV1(weapon::Spec spec,std::optional<weapon::Observation> estimate) {
    return std::make_unique<VelocityNormalPnGuidanceV1>(std::move(spec),std::move(estimate));
}
void registerNativeGuidanceV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.guidance.registerFactory("fsim.guidance.velocity_normal_pn.v1","fsim.guidance.velocity_normal_pn.v1",
        [](const DescriptorV1& d,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<GuidanceProviderV1> {
            auto spec=ctx.spec;if(!weapon::configureGuidance(d,spec,e))return {};
            return nativeGuidanceProviderV1(std::move(spec),ctx.initialEstimate);
        },false,error);
}

} // namespace fsim::ms::components
