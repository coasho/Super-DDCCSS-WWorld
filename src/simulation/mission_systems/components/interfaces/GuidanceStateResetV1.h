#pragma once

namespace fsim::ms::components {
/// Optional private state reset at an existing native pose-reset boundary.
/// The original guidance provider vtables and public SDK remain unchanged.
class GuidanceStateResetV1 {
public:
    virtual ~GuidanceStateResetV1()=default;
    virtual void resetGuidanceStateV1() noexcept=0;
};
}
