#include "Providers.h"
#include "providers/Builtins.h"
#include <cmath>

namespace fsim::ms::components {
namespace {
constexpr const char* plantKinds[]={"autopilot","allocator","actuator","propulsion","aerodynamics"};
}
ProviderRegistryV1 builtInProvidersV1() {
    ProviderRegistryV1 registry;
    registerNativeSeekerV1(registry);registerNativeGuidanceV1(registry);
    registerNativeAutopilotV1(registry);registerNativeAllocatorV1(registry);
    registerNativeActuatorV1(registry);registerNativePropulsionV1(registry);
    registerNativeAerodynamicsV1(registry);registerNativeFuzeV1(registry);
    registerNativeWarheadV1(registry);return registry;
}
bool ProviderRegistryV1::validationProfile(const ProfileV1& input,ProfileV1& output,std::string& error) const {
    auto next=input;
    if(!input.explicitProfile()) {output=std::move(next);error.clear();return true;}
    if(input.schemaVersion!=1 || input.components.size()!=9) {error="provider composition needs the nine-role version 1 profile";return false;}
    const auto select=[&](const char* kind,const auto& table) {
        const auto found=next.components.find(kind);
        if(found==next.components.end()) {error=std::string("provider role missing: ")+kind;return false;}
        if(found->second.availability!=AvailabilityV1::Available)return true;
        DescriptorV1 d;if(!table.descriptor(found->second,d,error))return false;
        found->second=std::move(d);return true;
    };
    if(!select("seeker",seeker) || !select("guidance",guidance) || !select("fuze",fuze) || !select("warhead",warhead))return false;
    if(!select("autopilot",autopilot) || !select("allocator",allocator) || !select("actuator",actuator) ||
        !select("propulsion",propulsion) || !select("aerodynamics",aerodynamics))return false;
    output=std::move(next);error.clear();return true;
}
bool ProviderRegistryV1::createPlant(const ProfileV1& profile,const ProviderContextV1& context,
    const weapon::PlantAdapterV1& selected,PlantProvidersV1& output,std::string& error) const {
    PlantProvidersV1 next;
    const auto make=[&](std::size_t i,const auto& table,auto& instance) {
        if(!selected.required[i] || selected.availability[i]!=AvailabilityV1::Available)return true;
        const auto* d=profile.find(plantKinds[i]);if(!d){error="native plant descriptor missing";return false;}
        instance=table.create(*d,context,error);if(!instance)return false;
        if(instance->nativeModel()!=context.card.model){error="native provider returned a different model binding";return false;}
        return true;
    };
    if(!make(0,autopilot,next.autopilot) || !make(1,allocator,next.allocator) || !make(2,actuator,next.actuator) ||
        !make(3,propulsion,next.propulsion) || !make(4,aerodynamics,next.aerodynamics))return false;
    output=std::move(next);error.clear();return true;
}
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
}
