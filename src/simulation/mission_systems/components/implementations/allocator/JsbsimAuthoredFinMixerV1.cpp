#include "JsbsimAuthoredFinMixerV1.h"
#include <utility>
#include <cmath>

namespace fsim::ms::components {

JsbsimAuthoredFinMixerV1::JsbsimAuthoredFinMixerV1(std::string model) : NativePlantBindingV1(std::move(model)) {}

weapon::FinCommands JsbsimAuthoredFinMixerV1::allocate(weapon::FinCommands input,const weapon::ControlStopsV1&) const noexcept {
    // JSBSim's authored mixer consumes these virtual channels. Do not
    // duplicate mixing or infer a replacement physical-fin allocator here.
    if(!input.available || !std::isfinite(input.aileron) || !std::isfinite(input.elevator) || !std::isfinite(input.rudder) ||
        std::abs(input.aileron)>1 || std::abs(input.elevator)>1 || std::abs(input.rudder)>1)return {};
    return input;
}

} // namespace fsim::ms::components
