#include "JsbsimDuctedRocketPropulsionV1.h"
#include <utility>

namespace fsim::ms::components {
JsbsimDuctedRocketPropulsionV1::JsbsimDuctedRocketPropulsionV1(std::string model)
    :NativePlantBindingV1(std::move(model)) {}
bool JsbsimDuctedRocketPropulsionV1::acceptsEngineCount(unsigned count) const noexcept {return count==2;}
bool JsbsimDuctedRocketPropulsionV1::burning(double nativeThrustN) const noexcept {return nativeThrustN>.1;}
}
