#pragma once
#include "simulation/mission_systems/weapon/OperatingPointControlV1.h"
namespace fsim::ms::components {
/// Additive sibling; retained AutopilotProviderV1 vtable/layout is unchanged.
class OperatingPointControlV1 {
public:
    virtual ~OperatingPointControlV1()=default;
    virtual weapon::OperatingPointResultV1 commandsOperatingPointV1(const weapon::OperatingPointInputV1&) const noexcept=0;
};
}
