#include "SweptPathFuzeV1.h"
#include <utility>

namespace fsim::ms::weapon {

FuzeResult SweptPathFuzeV1::evaluateProximity(const std::vector<MotionSample>& own,const std::vector<MotionSample>& target,
    double ageBeforeS,double durationS,double eligibleEndS,bool exclusiveEnd,std::uint32_t targetId) {
    (void)targetId;
    return sweptPathFuze(own,target,ageBeforeS,durationS,spec_,eligibleEndS,exclusiveEnd);
}

} // namespace fsim::ms::weapon
