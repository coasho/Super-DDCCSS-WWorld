#include "JsbsimDuctedRocketPropulsionV1.h"
#include <utility>
#include <cmath>

namespace fsim::ms::components {
JsbsimDuctedRocketPropulsionV1::JsbsimDuctedRocketPropulsionV1(std::string model)
    :NativePlantBindingV1(std::move(model)) {}
bool JsbsimDuctedRocketPropulsionV1::acceptsEngineCount(unsigned count) const noexcept {return count==2;}
bool JsbsimDuctedRocketPropulsionV1::burning(double nativeThrustN) const noexcept {return nativeThrustN>.1;}
JsbsimDuctedRocketPropulsionV2::JsbsimDuctedRocketPropulsionV2(std::string model):NativePlantBindingV1(std::move(model)) {}
bool JsbsimDuctedRocketPropulsionV2::acceptsEngineCount(unsigned count) const noexcept {return count==2;}
bool JsbsimDuctedRocketPropulsionV2::burning(double thrust) const noexcept {return std::isfinite(thrust)&&thrust>.1;}
}
