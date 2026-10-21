#pragma once

#include "simulation/mission_systems/components/support/CapturedFuzeLifecycleV1.h"

namespace fsim::ms::weapon {

class SweptPathFuzeV1 final : public CapturedFuzeLifecycleV1 {
public:
    using CapturedFuzeLifecycleV1::CapturedFuzeLifecycleV1;
    FuzeResult evaluateProximity(const std::vector<MotionSample>& own,const std::vector<MotionSample>& target,
        double ageBeforeS,double durationS,double eligibleEndS,bool exclusiveEnd,std::uint32_t targetId) override;
};

} // namespace fsim::ms::weapon
