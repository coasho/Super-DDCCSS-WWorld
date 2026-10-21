#pragma once
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
namespace fsim::ms::components {
class ImagingIrSeekerV2 final : public MeasuredSeekerV2 {
public:
    explicit ImagingIrSeekerV2(weapon::Spec);
    SeekerResultV2 sample(const SeekerFrameV2&) override;
};
}
