#pragma once
#include "ReceiverConfigurationV1.h"
#include "simulation/mission_systems/components/interfaces/ThermalReticleSeekerExecutionV1.h"

namespace fsim::ms::components {
struct ReticleReceiverConfigurationV1 {
    ThermalReticleConfigV1 policy;
    ReceiverScenePolicyV1 scene;
};
/// Additive envelope: a reticle is never represented as an old quadrant or
/// image kind. Existing V1/V2 layouts, enum values and admission are retained.
struct ReceiverConfigurationV3 {
    ReceiverConfigurationV2 inherited;
    std::optional<ReticleReceiverConfigurationV1> reticle;
};
bool receiverConfigurationV3(const DescriptorV1&,DescriptorV1& scalar,
    ReceiverConfigurationV3&,std::string& error);
}
