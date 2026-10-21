#include "NativePlantBinding.h"
namespace fsim::ms::components {
namespace {
class JsbsimSolidRocketPropulsionV1 final : public NativePlantBindingV1<PropulsionProviderV1> {
public:
    explicit JsbsimSolidRocketPropulsionV1(std::string model) : NativePlantBindingV1(std::move(model)) {}
    bool acceptsEngineCount(unsigned count) const noexcept override {return count==1;}
    bool burning(double nativeThrustN) const noexcept override {return nativeThrustN>.1;}
};
}
void registerNativePropulsionV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.propulsion.registerFactory("fsim.propulsion.jsbsim_generated_solid_rocket","fsim.propulsion.jsbsim_generated_solid_rocket",
        [](const DescriptorV1&,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<PropulsionProviderV1> {
            e.clear();return std::make_unique<JsbsimSolidRocketPropulsionV1>(ctx.card.model);
        },false,error);
}
}
