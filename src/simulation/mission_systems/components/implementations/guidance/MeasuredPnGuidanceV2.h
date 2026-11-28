#pragma once
#include <cstdint>
#include "simulation/mission_systems/components/interfaces/GuidanceProviderV1.h"
#include "simulation/mission_systems/components/interfaces/MeasuredGuidanceV2.h"
#include "simulation/mission_systems/components/interfaces/GuidanceStateResetV1.h"
namespace fsim::ms::components {
class MeasuredPnGuidanceV2 final : public GuidanceProviderV1, public MeasuredGuidanceV2, public GuidanceStateResetV1 {
public:
    explicit MeasuredPnGuidanceV2(weapon::Spec);
    weapon::Spec spec() const noexcept override;
    weapon::Output update(const weapon::Input&) override;
    weapon::Output updateMeasured(const SeekerFrameV2&,const SeekerResultV2&) override;
    void resetGuidanceStateV1() noexcept override {
        terminalAcquiredV3_=false;
        lastRateTrackId_=0;
        lastRateTimeS_=-1.;
        lastRateDirection_={};
        lastRate_={};
    }
private:
    weapon::Spec spec_;
    bool terminalAcquiredV3_=false;
    std::uint64_t lastRateTrackId_=0;
    double lastRateTimeS_=-1.;
    weapon::V3 lastRateDirection_{};
    weapon::V3 lastRate_{};
};
}
