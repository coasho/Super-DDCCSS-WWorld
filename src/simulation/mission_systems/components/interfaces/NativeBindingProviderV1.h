#pragma once

#include "simulation/mission_systems/weapon/PlantAdapter.h"
#include <string_view>

namespace fsim::ms::components {

class NativeBindingProviderV1 {
public:
    virtual ~NativeBindingProviderV1() = default;
    virtual std::string_view nativeModel() const noexcept = 0;
    virtual bool available(const weapon::NativePlantMetadataV1& expected,
                           const weapon::NativePlantMetadataV1& actual) const noexcept = 0;
};

} // namespace fsim::ms::components
