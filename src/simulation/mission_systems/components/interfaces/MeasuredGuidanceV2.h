#pragma once
#include "SeekerExecutionV2.h"

namespace fsim::ms::components {
class MeasuredGuidanceV2 {
public:
    virtual ~MeasuredGuidanceV2()=default;
    virtual weapon::Output updateMeasured(const SeekerFrameV2&,const SeekerResultV2&)=0;
};
}
