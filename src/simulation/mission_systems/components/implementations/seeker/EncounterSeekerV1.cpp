#include "EncounterSeekerV1.h"
#include <utility>

namespace fsim::ms::components {

EncounterSeekerV1::EncounterSeekerV1(weapon::SeekerConfigV1 config) : config_(config) {}

const weapon::SeekerConfigV1& EncounterSeekerV1::config() const noexcept {return config_;}

double EncounterSeekerV1::infraredRange(double base,double aspect,bool burner) const noexcept {
    return weapon::infraredAcquisitionRangeM(base,aspect,burner,config_);
}

double EncounterSeekerV1::radarRange(double base,double cross,bool modelled) const noexcept {
    return weapon::radarAcquisitionRangeM(base,cross,modelled,config_);
}

bool EncounterSeekerV1::notched(double vertical,double radial) const noexcept {return weapon::radarNotched(vertical,radial,config_);}

double EncounterSeekerV1::decoySignal(double age,bool infrared) const noexcept {return weapon::decoySignal(age,infrared,config_);}

bool EncounterSeekerV1::illuminated(const double direction[3],double azimuth,double elevation,double aw,double ew) const noexcept {
    return weapon::withinIlluminationBeam(direction,azimuth,elevation,aw,ew,config_);
}

} // namespace fsim::ms::components
