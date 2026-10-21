#include "VelocityNormalPnGuidanceV1.h"
#include <utility>

namespace fsim::ms::components {

VelocityNormalPnGuidanceV1::VelocityNormalPnGuidanceV1(weapon::Spec spec,std::optional<weapon::Observation> estimate) : native_(std::move(spec),std::move(estimate)) {}

weapon::Spec VelocityNormalPnGuidanceV1::spec() const noexcept {return native_.spec();}

weapon::Output VelocityNormalPnGuidanceV1::update(const weapon::Input& input) {return native_.update(input);}

} // namespace fsim::ms::components
