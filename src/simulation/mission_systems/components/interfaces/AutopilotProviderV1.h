#pragma once

#include "NativeBindingProviderV1.h"

namespace fsim::ms::components {

class AutopilotProviderV1 : public NativeBindingProviderV1 {
public:
    virtual weapon::FinCommands commands(const weapon::AutopilotInput&,
                                         const weapon::ControlStopsV1&) const noexcept = 0;
};

} // namespace fsim::ms::components
