#pragma once

#include "simulation/mission_systems/weapon/Seeker.h"

namespace fsim::ms::components {

class SeekerProviderV1 {
public:
    virtual ~SeekerProviderV1() = default;
    virtual const weapon::SeekerConfigV1& config() const noexcept = 0;
    virtual double infraredRange(double base, double aspect, bool afterburner) const noexcept = 0;
    virtual double radarRange(double base, double crossSection, bool modelled) const noexcept = 0;
    virtual bool notched(double vertical, double radialSpeed) const noexcept = 0;
    virtual double decoySignal(double age, bool infrared) const noexcept = 0;
    virtual bool illuminated(const double direction[3], double azimuth, double elevation,
                             double azimuthWidth, double elevationWidth) const noexcept = 0;
};

} // namespace fsim::ms::components
