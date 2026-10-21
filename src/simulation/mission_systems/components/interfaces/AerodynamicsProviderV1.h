#pragma once

#include "NativeBindingProviderV1.h"

namespace fsim::ms::components {

class AerodynamicsProviderV1 : public NativeBindingProviderV1 {
public:
    virtual std::string_view coefficientSource() const noexcept = 0;
};

} // namespace fsim::ms::components
