#include "BuiltInProvidersV1.h"

namespace fsim::ms::components {

ProviderRegistryV1 builtInProvidersV1() {
    ProviderRegistryV1 registry;
    registerNativeSeekerV1(registry);registerNativeGuidanceV1(registry);
    registerNativeAutopilotV1(registry);registerNativeAllocatorV1(registry);
    registerNativeActuatorV1(registry);registerNativePropulsionV1(registry);
    registerNativeAerodynamicsV1(registry);registerNativeFuzeV1(registry);
    registerNativeWarheadV1(registry);return registry;
}

} // namespace fsim::ms::components
