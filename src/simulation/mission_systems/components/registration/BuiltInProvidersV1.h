#pragma once
#include "simulation/mission_systems/components/ProviderRegistryV1.h"

namespace fsim::ms::components {
ProviderRegistryV1 builtInProvidersV1();
void registerNativeSeekerV1(ProviderRegistryV1&);
void registerNativeGuidanceV1(ProviderRegistryV1&);
void registerNativeAutopilotV1(ProviderRegistryV1&);
void registerNativeAllocatorV1(ProviderRegistryV1&);
void registerNativeActuatorV1(ProviderRegistryV1&);
void registerNativePropulsionV1(ProviderRegistryV1&);
void registerNativeAerodynamicsV1(ProviderRegistryV1&);
void registerNativeFuzeV1(ProviderRegistryV1&);
void registerNativeWarheadV1(ProviderRegistryV1&);
std::unique_ptr<SeekerProviderV1> nativeSeekerProviderV1(weapon::SeekerConfigV1);
std::unique_ptr<GuidanceProviderV1> nativeGuidanceProviderV1(weapon::Spec,std::optional<weapon::Observation>);
}
