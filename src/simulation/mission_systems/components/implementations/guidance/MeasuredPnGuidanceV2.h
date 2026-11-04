#pragma once
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
    void resetGuidanceStateV1() noexcept override {terminalAcquiredV3_=false;}
private:
    weapon::Spec spec_;
    bool terminalAcquiredV3_=false;
};
}
