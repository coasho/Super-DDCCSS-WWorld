#pragma once
#include "simulation/mission_systems/components/Profile.h"
#include "simulation/mission_systems/components/interfaces/ThermalSeekerExecutionV1.h"
#include "simulation/mission_systems/components/interfaces/RadioSeekerExecutionV1.h"
#include "simulation/mission_systems/components/interfaces/ThermalDualBandSeekerExecutionV1.h"

namespace fsim::ms::components {
enum class PhysicalReceiverKindV1 {Scalar=0,Quadrant=1,Image=2,Radio=3};
struct ReceiverScenePolicyV1 {
    double lowUm=0,highUm=0,extinctionPerM=0;
    double skyTemperatureK=0,groundTemperatureK=0,bodyEmissivity=0,skinRecoveryFactor=0;
    double engineHeatFraction=0,exhaustEmissivity=0,exhaustAreaScale=0;
    double flareTemperatureK=0,flareEmissivity=0,flareInitialRadiusM=0,flareExpansionMs=0,flareCoolingS=0;
    double groundSigma0Land=0,groundSigma0Sea=0,groundGrazingExponent=0,groundRangeM=0;
    unsigned groundRangeBands=0,groundAzimuthSamples=0;
};
struct ReceiverConfigurationV1 {
    PhysicalReceiverKindV1 kind=PhysicalReceiverKindV1::Scalar;
    ThermalImageConfigV1 thermal;
    RadioReceiverConfigV1 radio;
    ReceiverScenePolicyV1 scene;
};
/// Atomic extension admission and exact legacy-descriptor normalization.
/// Absence leaves every retained descriptor byte/value untouched. Unknown
/// roles/bindings remain for the existing strict legacy admission to reject.
bool receiverConfigurationV1(const DescriptorV1&,DescriptorV1& scalar,
    ReceiverConfigurationV1&,std::string& error);
/// Additive configuration envelope; retained V1 layouts and admission remain
/// unchanged. A dual receiver has two independently resolved detector policies
/// and a separate band-integrated scene policy for the second channel.
struct ReceiverConfigurationV2 {
    ReceiverConfigurationV1 single;
    std::optional<ThermalDualBandConfigV1> dual;
    std::optional<ReceiverScenePolicyV1> secondBandScene;
};
bool receiverConfigurationV2(const DescriptorV1&,DescriptorV1& scalar,
    ReceiverConfigurationV2&,std::string& error);
} // namespace fsim::ms::components
