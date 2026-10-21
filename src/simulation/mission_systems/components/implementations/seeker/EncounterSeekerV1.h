#pragma once

#include "simulation/mission_systems/components/interfaces/SeekerProviderV1.h"

namespace fsim::ms::components {

class EncounterSeekerV1 final : public SeekerProviderV1 {
public:
    explicit EncounterSeekerV1(weapon::SeekerConfigV1 config);
    const weapon::SeekerConfigV1& config() const noexcept override;
    double infraredRange(double base,double aspect,bool burner) const noexcept override;
    double radarRange(double base,double cross,bool modelled) const noexcept override;
    bool notched(double vertical,double radial) const noexcept override;
    double decoySignal(double age,bool infrared) const noexcept override;
    bool illuminated(const double direction[3],double azimuth,double elevation,double aw,double ew) const noexcept override;
private:
    weapon::SeekerConfigV1 config_;
};

} // namespace fsim::ms::components
