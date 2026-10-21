#pragma once

#include "Autopilot.h"
#include "simulation/mission_systems/components/Profile.h"
#include <array>
#include <cstdint>
#include <limits>

namespace fsim::ms::weapon {
struct NativePlantMetadataV1 {
    double schemaVersion=std::numeric_limits<double>::quiet_NaN();
    double implementationCode=std::numeric_limits<double>::quiet_NaN();
    double fingerprint=std::numeric_limits<double>::quiet_NaN();
};
/// Actual loaded native properties, in autopilot/allocator/actuator/propulsion/aerodynamics order.
struct NativePlantIdentityV1 { std::array<NativePlantMetadataV1,5> components; };
struct PlantAdapterV1 {
    bool configured=false,hasControl=false;
    FinControlConfigV1 control;
    std::array<components::AvailabilityV1,5> availability{};
    std::array<NativePlantMetadataV1,5> expected;
    std::array<unsigned,5> implementations{};
    std::array<bool,5> required{{true,true,true,true,true}};
};
std::uint32_t plantFingerprintV1(const components::DescriptorV1&) noexcept;
bool configurePlantV1(const components::DescriptorV1& autopilot,const components::DescriptorV1& allocator,
    const components::DescriptorV1& actuator,const components::DescriptorV1& propulsion,
    const components::DescriptorV1& aerodynamics,const StoreCard& card,PlantAdapterV1& output,std::string& error);
/// Select only ballistic aerodynamics. The original full ProfileV1 remains
/// separately owned and unchanged; omitted providers are not declared present.
bool configureBallisticPlantV1(const components::DescriptorV1& aerodynamics,
    const StoreCard& card,PlantAdapterV1& output,std::string& error);
/// A non-inert release without guidance still executes its declared native
/// propulsion. A genuinely unpowered body may declare it not applicable.
bool configureUnguidedPlantV1(const components::DescriptorV1& propulsion,
    const components::DescriptorV1& aerodynamics,const StoreCard& card,PlantAdapterV1& output,std::string& error);
bool plantAvailableV1(const PlantAdapterV1&,const NativePlantIdentityV1&) noexcept;
bool plantCommandInputsAvailableV1(const PlantAdapterV1&,const AutopilotInput&,const ControlStopsV1&,
                                  const NativePlantIdentityV1&) noexcept;
FinCommands plantCommandsV1(const PlantAdapterV1&,const AutopilotInput&,const ControlStopsV1&,
                            const NativePlantIdentityV1&) noexcept;
}
