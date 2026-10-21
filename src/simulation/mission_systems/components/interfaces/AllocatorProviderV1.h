#pragma once

#include "NativeBindingProviderV1.h"

namespace fsim::ms::components {

class AllocatorProviderV1 : public NativeBindingProviderV1 {
public:
    virtual weapon::FinCommands allocate(weapon::FinCommands,
                                         const weapon::ControlStopsV1&) const noexcept = 0;
};

} // namespace fsim::ms::components
