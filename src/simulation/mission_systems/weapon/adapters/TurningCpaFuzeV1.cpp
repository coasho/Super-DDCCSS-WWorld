#include "CapturedFuzeLifecycle.h"
namespace fsim::ms::weapon {
namespace {
class TurningCpaFuzeV1 final : public CapturedFuzeLifecycleV1 {
public:
    using CapturedFuzeLifecycleV1::CapturedFuzeLifecycleV1;
    FuzeResult evaluateProximity(const std::vector<MotionSample>& own,const std::vector<MotionSample>& target,
        double ageBeforeS,double durationS,double eligibleEndS,bool exclusiveEnd,std::uint32_t targetId) override {
        (void)targetId;
        return turn_.evaluate(own,target,ageBeforeS,durationS,spec_,eligibleEndS,exclusiveEnd,targetId);
    }
};
}
std::unique_ptr<FuzeAdapter> makeTurningCpaFuzeV1(std::string id,Spec spec) {
    return std::make_unique<TurningCpaFuzeV1>(std::move(id),std::move(spec));
}
}
