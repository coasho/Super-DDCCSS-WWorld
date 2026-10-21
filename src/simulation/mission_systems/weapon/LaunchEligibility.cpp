#include "simulation/mission_systems/weapon/LaunchEligibility.h"

#include "foundation/core/Units.h"
#include "simulation/mission_systems/Cards.h"
#include "simulation/mission_systems/Model.h"
#include "simulation/mission_systems/weapon/Seeker.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace fsim::ms::weapon {

LaunchEligibility checkLaunchEligibility(const StoreCard& card, const sim::VehicleState& own,
                                        const sim::VehicleState* target) {
    if (!target) return {Reason::UnknownTrack, false};
    double ray[3], body[3];
    for (int k = 0; k < 3; ++k) ray[k] = target->positionEcef[k] - own.positionEcef[k];
    geom::toBody(own, ray, body);
    const double off = std::acos(std::clamp(body[0] / std::max(geom::norm3(ray), 1.0), -1.0, 1.0));
    const bool heat = card.seeker.find("infrared") != std::string::npos;
    if (off > card.value("launch_fov_deg", heat ? 25.0 : 55.0) * units::kPi / 180.0)
        return {Reason::OutOfReach, false};
    return {Reason::None, card.seeker == "semi_active_radar"};
}

Reason checkLaunchIllumination(std::uint32_t launcher, const IlluminationFacts& facts) noexcept {
    if (!facts.designated || facts.owner != launcher || !facts.localSource
        || !facts.illuminating || facts.radarTrack != facts.track)
        return Reason::UnknownTrack;
    return Reason::None;
}

LaunchEligibility checkLaunchEligibility(const StoreCard& card,const sim::VehicleState& own,
    const sim::VehicleState* target,const SeekerConfigV1& config) {
    if (config.version==0) return checkLaunchEligibility(card,own,target);
    if (!validSeekerConfigV1(config)) return {Reason::InvalidParameter,false};
    if (!target) return {Reason::UnknownTrack,false};
    double ray[3],body[3];
    for (int k=0;k<3;++k) ray[k]=target->positionEcef[k]-own.positionEcef[k];
    geom::toBody(own,ray,body);
    const double off=std::acos(std::clamp(body[0]/std::max(geom::norm3(ray),1.0),-1.0,1.0));
    if (off>config.launchFovRad) return {Reason::OutOfReach,false};
    return {Reason::None,config.mode==SeekerModeV1::SemiActiveRadar};
}

} // namespace fsim::ms::weapon
