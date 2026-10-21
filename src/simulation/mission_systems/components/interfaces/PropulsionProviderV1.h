#pragma once

#include "NativeBindingProviderV1.h"

namespace fsim::ms::components {

class PropulsionProviderV1 : public NativeBindingProviderV1 {
public:
    virtual bool acceptsEngineCount(unsigned) const noexcept = 0;
    virtual bool burning(double nativeThrustN) const noexcept = 0;
};

} // namespace fsim::ms::components
