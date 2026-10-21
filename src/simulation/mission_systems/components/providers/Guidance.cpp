#include "Builtins.h"
namespace fsim::ms::components {
namespace {
class VelocityNormalPnGuidanceV1 final : public GuidanceProviderV1 {
public:
    VelocityNormalPnGuidanceV1(weapon::Spec spec,std::optional<weapon::Observation> estimate) : native_(std::move(spec),std::move(estimate)) {}
    weapon::Spec spec() const noexcept override {return native_.spec();}
    weapon::Output update(const weapon::Input& input) override {return native_.update(input);}
private:
    weapon::Guidance native_;
};
}
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
}
