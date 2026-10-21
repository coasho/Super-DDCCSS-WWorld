#include "PlantProvidersV1.h"
#include <cmath>

namespace fsim::ms::components {

bool PlantProvidersV1::available(const weapon::PlantAdapterV1& selected,const weapon::NativePlantIdentityV1& actual) const noexcept {
    if(!weapon::plantAvailableV1(selected,actual))return false;
    const NativeBindingProviderV1* roles[]={autopilot.get(),allocator.get(),actuator.get(),propulsion.get(),aerodynamics.get()};
    for(std::size_t i=0;i<5;++i)if(selected.required[i] && selected.availability[i]==AvailabilityV1::Available &&
        (!roles[i] || !roles[i]->available(selected.expected[i],actual.components[i])))return false;
    return true;
}
weapon::FinCommands PlantProvidersV1::commands(const weapon::PlantAdapterV1& selected,const weapon::AutopilotInput& input,
    const weapon::ControlStopsV1& stops,const weapon::NativePlantIdentityV1& actual) const noexcept {
    if(!available(selected,actual) || !weapon::plantCommandInputsAvailableV1(selected,input,stops,actual))return {};
    if(!autopilot || !allocator || !actuator || !actuator->accepts(input,stops))return {};
    auto output=autopilot->commands(input,stops);
    const auto valid=[](const auto& command) {return command.available && std::isfinite(command.aileron) &&
        std::isfinite(command.elevator) && std::isfinite(command.rudder) && std::abs(command.aileron)<=1 &&
        std::abs(command.elevator)<=1 && std::abs(command.rudder)<=1;};
    if(!valid(output))return {};
    output=allocator->allocate(output,stops);
    return valid(output) ? output : weapon::FinCommands{};
}

} // namespace fsim::ms::components
