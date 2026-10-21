#include "CapturedFuzeLifecycle.h"
namespace fsim::ms::weapon {
namespace {
class SweptPathFuzeV1 final : public CapturedFuzeLifecycleV1 {
public:
    using CapturedFuzeLifecycleV1::CapturedFuzeLifecycleV1;
    FuzeResult evaluateProximity(const std::vector<MotionSample>& own,const std::vector<MotionSample>& target,
        double ageBeforeS,double durationS,double eligibleEndS,bool exclusiveEnd,std::uint32_t targetId) override {
        (void)targetId;
        return sweptPathFuze(own,target,ageBeforeS,durationS,spec_,eligibleEndS,exclusiveEnd);
    }
};
}
std::unique_ptr<FuzeAdapter> makeSweptPathFuzeV1(std::string id,Spec spec) {
    return std::make_unique<SweptPathFuzeV1>(std::move(id),std::move(spec));
}
}
