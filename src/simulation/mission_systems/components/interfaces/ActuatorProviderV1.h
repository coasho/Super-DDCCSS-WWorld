#pragma once

#include "NativeBindingProviderV1.h"

namespace fsim::ms::components {

class ActuatorProviderV1 : public NativeBindingProviderV1 {
public:
    virtual bool accepts(const weapon::AutopilotInput&,
                         const weapon::ControlStopsV1&) const noexcept = 0;
};

} // namespace fsim::ms::components
