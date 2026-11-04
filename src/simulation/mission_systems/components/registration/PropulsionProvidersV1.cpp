#include "BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/implementations/propulsion/JsbsimSolidRocketPropulsionV1.h"
#include "simulation/mission_systems/components/implementations/propulsion/JsbsimDuctedRocketPropulsionV1.h"
#include <cmath>

namespace fsim::ms::components {
namespace {
class JsbsimTurbofanV1 final : public NativePlantBindingV1<PropulsionProviderV1> {
public:
    explicit JsbsimTurbofanV1(std::string model):NativePlantBindingV1(std::move(model)){}
    bool acceptsEngineCount(unsigned count) const noexcept override{return count==1;}
    bool burning(double thrust) const noexcept override{return std::isfinite(thrust)&&thrust>.1;}
};
}

void registerNativePropulsionV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.propulsion.registerFactory("fsim.propulsion.jsbsim_turbofan.v1","fsim.propulsion.jsbsim_turbofan.v1",
        [](const DescriptorV1&,const ProviderContextV1& ctx,std::string& e)->std::unique_ptr<PropulsionProviderV1>{
            e.clear();return std::make_unique<JsbsimTurbofanV1>(ctx.card.model);
        },false,error);
    registry.propulsion.registerFactory("fsim.propulsion.jsbsim_generated_solid_rocket","fsim.propulsion.jsbsim_generated_solid_rocket",
        [](const DescriptorV1&,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<PropulsionProviderV1> {
            e.clear();return std::make_unique<JsbsimSolidRocketPropulsionV1>(ctx.card.model);
        },false,error);
    registry.propulsion.registerFactory("fsim.propulsion.jsbsim_commanded_dual_pulse.v1","fsim.propulsion.jsbsim_commanded_dual_pulse.v1",
        [](const DescriptorV1&,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<PropulsionProviderV1> {
            e.clear();return std::make_unique<JsbsimSolidRocketPropulsionV1>(ctx.card.model);
        },false,error);
    registry.propulsion.registerFactory("fsim.propulsion.jsbsim_ducted_rocket.v2","fsim.propulsion.jsbsim_ducted_rocket.v2",
        [](const DescriptorV1&,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<PropulsionProviderV1> {
            e.clear();return std::make_unique<JsbsimDuctedRocketPropulsionV2>(ctx.card.model);
        },false,error);
    registry.propulsion.registerFactory("fsim.propulsion.jsbsim_ducted_rocket.v1","fsim.propulsion.jsbsim_ducted_rocket.v1",
        [](const DescriptorV1&,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<PropulsionProviderV1> {
            e.clear();return std::make_unique<JsbsimDuctedRocketPropulsionV1>(ctx.card.model);
        },false,error);
}

} // namespace fsim::ms::components
