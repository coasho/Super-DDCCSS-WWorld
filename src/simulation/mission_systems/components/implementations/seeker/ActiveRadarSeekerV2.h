#pragma once
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
namespace fsim::ms::components {
class ActiveRadarSeekerV2 final : public MeasuredSeekerV2 {
public:
    struct PhasePreviewV1 {
        bool valid=false,active=false;
        double rangeM=0,activationRangeM=0;
        std::optional<LauncherEstimateV2> estimate,prediction;
    };
    ActiveRadarSeekerV2(weapon::Spec,std::optional<weapon::Observation> initialEstimate);
    /// Nonmutating private physical-receiver run condition. It shares exactly
    /// the retained family's estimate/activation calculation, without sampling
    /// twice or consuming the family's cached-epoch admission.
    PhasePreviewV1 previewPhaseV1(const SeekerFrameV2&) const;
    SeekerResultV2 sample(const SeekerFrameV2&) override;
    void resetTracking() noexcept override;
private:
    bool active_=false, initialEpoch_=true;
    std::optional<LauncherEstimateV2> estimate_;
};
}
