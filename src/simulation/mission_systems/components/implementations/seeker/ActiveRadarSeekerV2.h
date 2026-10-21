#pragma once
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
namespace fsim::ms::components {
class ActiveRadarSeekerV2 final : public MeasuredSeekerV2 {
public:
    ActiveRadarSeekerV2(weapon::Spec,std::optional<weapon::Observation> initialEstimate);
    SeekerResultV2 sample(const SeekerFrameV2&) override;
private:
    bool active_=false, initialEpoch_=true;
    std::optional<LauncherEstimateV2> estimate_;
};
}
