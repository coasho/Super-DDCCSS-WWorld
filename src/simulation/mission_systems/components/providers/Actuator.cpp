#include "NativePlantBinding.h"
#include <algorithm>
#include <cmath>
#include <limits>
namespace fsim::ms::components {
namespace {
class JsbsimLagRateStopsActuatorV1 final : public NativePlantBindingV1<ActuatorProviderV1> {
public:
    JsbsimLagRateStopsActuatorV1(std::string model,double lag,double rate) : NativePlantBindingV1(std::move(model)),lag_(lag),rate_(rate) {}
    bool accepts(const weapon::AutopilotInput& input,const weapon::ControlStopsV1& stops) const noexcept override {
        const auto same=[](double a,double b) {return std::isfinite(a)&&std::isfinite(b)&&a>0&&b>0&&
            std::abs(a-b)<=16*std::numeric_limits<double>::epsilon()*std::max(a,b);};
        return stops.actuatorVersion==1 && same(input.actuatorLagS,lag_) && same(stops.channelRateRadS,rate_) &&
            !stops.finRateRadS.empty() && std::all_of(stops.finRateRadS.begin(),stops.finRateRadS.end(),[&](double rate){return same(rate,rate_);});
    }
private:
    double lag_,rate_;
};
}
void registerNativeActuatorV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.actuator.registerFactory("fsim.actuator.jsbsim_lag_rate_stops","fsim.actuator.jsbsim_lag_rate_stops",
        [](const DescriptorV1& d,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<ActuatorProviderV1> {
            double lag=0,rate=0;
            if(!parameterV1(d,"lag_s","s",lag,e) || !parameterV1(d,"rate_rad_s","rad/s",rate,e))return {};
            return std::make_unique<JsbsimLagRateStopsActuatorV1>(ctx.card.model,lag,rate);
        },false,error);
}
}
