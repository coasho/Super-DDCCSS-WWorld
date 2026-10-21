#include "JsbsimSolidRocketPropulsionV1.h"
#include <utility>

namespace fsim::ms::components {

JsbsimSolidRocketPropulsionV1::JsbsimSolidRocketPropulsionV1(std::string model) : NativePlantBindingV1(std::move(model)) {}

bool JsbsimSolidRocketPropulsionV1::acceptsEngineCount(unsigned count) const noexcept {return count==1;}

bool JsbsimSolidRocketPropulsionV1::burning(double nativeThrustN) const noexcept {return nativeThrustN>.1;}

} // namespace fsim::ms::components
