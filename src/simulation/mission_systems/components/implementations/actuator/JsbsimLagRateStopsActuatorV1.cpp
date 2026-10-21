#include "JsbsimLagRateStopsActuatorV1.h"
#include <utility>
#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::ms::components {

JsbsimLagRateStopsActuatorV1::JsbsimLagRateStopsActuatorV1(std::string model,double lag,double rate) : NativePlantBindingV1(std::move(model)),lag_(lag),rate_(rate) {}

bool JsbsimLagRateStopsActuatorV1::accepts(const weapon::AutopilotInput& input,const weapon::ControlStopsV1& stops) const noexcept {
    const auto same=[](double a,double b) {return std::isfinite(a)&&std::isfinite(b)&&a>0&&b>0&&
        std::abs(a-b)<=16*std::numeric_limits<double>::epsilon()*std::max(a,b);};
    return stops.actuatorVersion==1 && same(input.actuatorLagS,lag_) && same(stops.channelRateRadS,rate_) &&
        !stops.finRateRadS.empty() && std::all_of(stops.finRateRadS.begin(),stops.finRateRadS.end(),[&](double rate){return same(rate,rate_);});
}

} // namespace fsim::ms::components
