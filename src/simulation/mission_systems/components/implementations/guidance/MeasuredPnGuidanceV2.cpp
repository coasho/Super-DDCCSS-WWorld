#include "MeasuredPnGuidanceV2.h"
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
namespace fsim::ms::components {
using namespace measured;
MeasuredPnGuidanceV2::MeasuredPnGuidanceV2(weapon::Spec s):spec_(std::move(s)) {}
weapon::Spec MeasuredPnGuidanceV2::spec() const noexcept {return spec_;}
weapon::Output MeasuredPnGuidanceV2::update(const weapon::Input&) {
    // The legacy Cartesian call cannot manufacture a sampled measurement.
    return {};
}
weapon::Output MeasuredPnGuidanceV2::updateMeasured(const SeekerFrameV2& f,const SeekerResultV2& r) {
    weapon::Output out;
    if(!r.valid||!finite(f.position)||!finite(f.velocity)||!finite(f.up)||
       !std::isfinite(f.ageS)||f.ageS<0||!std::isfinite(spec_.maxG)||spec_.maxG<=0||
       !std::isfinite(spec_.navigationGain)||spec_.navigationGain<=2)return out;
    out.active=r.active;out.tracking=r.tracking;
    out.supported=r.supported&&(spec_.seeker=="semi_active_radar"||spec_.launcherUpdates);
    out.holding=r.track?r.track->localId:0;
    out.phase=r.tracking?WeaponPhase::Homing:out.holding?WeaponPhase::Lost:
        r.midcourseEstimate?WeaponPhase::Midcourse:WeaponPhase::Seeking;
    if(r.timeToActiveS)out.timeToActiveS=*r.timeToActiveS;
    const auto speed=norm(f.velocity);if(!std::isfinite(speed))return {};
    const auto heading=unit(f.velocity);weapon::V3 steering{};
    if(r.tracking&&r.track) {
        const auto& t=*r.track;
        if(!finite(t.direction)||!finite(t.directionRate))return {};
        // IR/SARH use own speed and measured LOS rate. ARH uses only its
        // admitted range-rate measurement, never hidden target velocity.
        double scaleSpeed=speed;
        if(t.rangeRateMs) {
            if(!std::isfinite(*t.rangeRateMs))return {};
            scaleSpeed=std::max(0.,-*t.rangeRateMs);
            if(t.rangeM&&scaleSpeed>0)out.timeToGoS=*t.rangeM/scaleSpeed;
        }
        if(t.rateAvailable)steering=scale(cross(cross(t.direction,t.directionRate),heading),spec_.navigationGain*scaleSpeed);
    } else if(r.midcourseEstimate&&spec_.inertialMidcourse) {
        const auto relative=subtract(r.midcourseEstimate->position,f.position);
        const auto velocity=subtract(r.midcourseEstimate->velocity,f.velocity);
        const auto range=norm(relative);
        if(!finite(relative)||!finite(velocity)||!std::isfinite(range))return {};
        if(range>0) {
            const auto closing=-dot(velocity,unit(relative));
            const auto angular=scale(cross(unit(relative),velocity),1/range);
            if(closing>0) {
                steering=scale(cross(angular,heading),spec_.navigationGain*closing);
                out.timeToGoS=range/closing;
            }
        }
        if(out.phase==WeaponPhase::Midcourse) {
            const auto loft=std::sin(std::clamp(spec_.loftRad,0.,.7))*spec_.loftAccelerationG*9.80665;
            const auto up=unit(f.up);steering=add(steering,scale(subtract(up,scale(heading,dot(up,heading))),loft));
        }
    }
    steering=subtract(steering,scale(heading,dot(steering,heading)));
    const auto command=add(steering,scale(unit(f.up),9.80665));
    const auto magnitude=norm(command),limit=spec_.maxG*9.80665;
    if(!finite(command)||!std::isfinite(magnitude)||!std::isfinite(limit))return {};
    out.acceleration=magnitude>limit&&magnitude>0?scale(command,limit/magnitude):command;
    return out;
}
}
