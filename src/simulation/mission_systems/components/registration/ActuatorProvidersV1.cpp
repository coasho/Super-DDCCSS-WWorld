#include "BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/implementations/actuator/JsbsimLagRateStopsActuatorV1.h"

namespace fsim::ms::components {

void registerNativeActuatorV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.actuator.registerFactory("fsim.actuator.jsbsim_lag_rate_stops","fsim.actuator.jsbsim_lag_rate_stops",
        [](const DescriptorV1& d,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<ActuatorProviderV1> {
            double lag=0,rate=0;
            if(!parameterV1(d,"lag_s","s",lag,e) || !parameterV1(d,"rate_rad_s","rad/s",rate,e))return {};
            return std::make_unique<JsbsimLagRateStopsActuatorV1>(ctx.card.model,lag,rate);
        },false,error);
}

} // namespace fsim::ms::components
