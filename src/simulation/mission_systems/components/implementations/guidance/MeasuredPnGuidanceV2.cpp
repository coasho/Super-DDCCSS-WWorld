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
    const bool finiteReceiver=spec_.seekerModel.engineering.version==3;
    if(finiteReceiver) {
        if(!std::isfinite(f.timeS)||f.timeS<0)return {};
        if(r.tracking) {
            if(!r.track)return {};
            const auto& t=*r.track;
            if(!t.localId||!std::isfinite(t.sampleTimeS)||t.sampleTimeS<0||t.sampleTimeS>f.timeS||
               !finite(t.direction)||!(norm(t.direction)>0)||!finite(t.directionRate)||
               (t.rangeM&&(!std::isfinite(*t.rangeM)||*t.rangeM<spec_.seekerModel.minimumRangeM))||
               (t.rangeRateMs&&!std::isfinite(*t.rangeRateMs)))return {};
        }
    }
    const bool terminal=terminalAcquiredV3_||(finiteReceiver&&r.tracking);
    out.active=r.active;out.tracking=r.tracking;
    out.supported=r.supported&&(spec_.seeker=="semi_active_radar"||spec_.launcherUpdates);
    out.holding=r.track?r.track->localId:0;
    out.phase=r.tracking?WeaponPhase::Homing:(terminal||out.holding)?WeaponPhase::Lost:
        r.midcourseEstimate?WeaponPhase::Midcourse:WeaponPhase::Seeking;
    if(r.timeToActiveS)out.timeToActiveS=*r.timeToActiveS;
    const auto speed=norm(f.velocity);if(!std::isfinite(speed))return {};
    const auto heading=unit(f.velocity);weapon::V3 steering{};
    // Bearing-only seekers (IR/SARH) retain their last measured LOS/rate inside
    // authored track_memory_s during transient missed dwells, and conscan
    // reticles may emit a transient coarse/unresolved dwell between precise
    // captures. Coast on the last admitted same-track measured LOS rate with
    // smooth exponential decay over a brief dropout window (<= 0.18 s) instead
    // of dropping instantly to 0G or holding stale high-G commands past CPA.
    if(r.tracking&&r.track&&r.track->rateAvailable&&r.track->localId!=0&&
       std::isfinite(r.track->sampleTimeS)&&r.track->sampleTimeS>=0&&r.track->sampleTimeS<=f.timeS&&
       finite(r.track->direction)&&norm(r.track->direction)>0&&finite(r.track->directionRate)) {
        lastRateTrackId_=r.track->localId;
        lastRateTimeS_=r.track->sampleTimeS;
        lastRateDirection_=r.track->direction;
        lastRate_=r.track->directionRate;
    } else if(!r.track||r.track->localId!=lastRateTrackId_) {
        lastRateTrackId_=0;
        lastRateTimeS_=-1.;
    }
    const double coastAge=(r.track&&std::isfinite(r.track->sampleTimeS)&&std::isfinite(f.timeS))?(f.timeS-r.track->sampleTimeS):-1.;
    const bool coastingBearing=!r.tracking&&r.track&&r.track->rateAvailable&&!r.midcourseEstimate&&
        coastAge>=0.&&coastAge<=0.18;
    const double coarseCoastAge=(lastRateTimeS_>=0.&&std::isfinite(f.timeS))?(f.timeS-lastRateTimeS_):-1.;
    const bool coastingCoarse=r.tracking&&r.track&&!r.track->rateAvailable&&!r.midcourseEstimate&&
        lastRateTrackId_!=0&&r.track->localId==lastRateTrackId_&&
        coarseCoastAge>=0.&&coarseCoastAge<=0.35;
    if((r.tracking||coastingBearing)&&r.track) {
        auto t=*r.track;
        if(!finite(t.direction)||!finite(t.directionRate))return {};
        if(coastingCoarse) {
            t.sampleTimeS=lastRateTimeS_;
            t.direction=lastRateDirection_;
            t.directionRate=lastRate_;
            t.rateAvailable=true;
        }
        if(r.measurementTimeS||coastingBearing||coastingCoarse) {
            if(!std::isfinite(t.sampleTimeS)||t.sampleTimeS>f.timeS)return {};
            t.direction=unit(add(t.direction,scale(t.directionRate,f.timeS-t.sampleTimeS)));
        }
        // IR/SARH use own speed and measured LOS rate. ARH uses only its
        // admitted range-rate measurement, never hidden target velocity.
        double scaleSpeed=speed;
        if(t.rangeRateMs) {
            if(!std::isfinite(*t.rangeRateMs))return {};
            scaleSpeed=std::max(0.,-*t.rangeRateMs);
            if(t.rangeM&&scaleSpeed>0)out.timeToGoS=*t.rangeM/scaleSpeed;
        }
        const double coastDecay=coastingBearing?std::exp(-coastAge/0.08):
            (coastingCoarse?std::exp(-coarseCoastAge/0.25):1.);
        if(t.rateAvailable)steering=scale(cross(cross(t.direction,t.directionRate),heading),spec_.navigationGain*scaleSpeed*coastDecay);
    } else if(r.midcourseEstimate&&spec_.inertialMidcourse) {
        const auto& estimate=*r.midcourseEstimate;
        if(!std::isfinite(estimate.sampleTimeS)||estimate.sampleTimeS<0||estimate.sampleTimeS>f.timeS)return {};
        const auto predicted=add(estimate.position,scale(estimate.velocity,f.timeS-estimate.sampleTimeS));
        const auto relative=subtract(predicted,f.position);
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
        if(out.phase==WeaponPhase::Midcourse&&spec_.loftRad>0&&spec_.loftAccelerationG>0) {
            const auto up=unit(f.up);
            const double targetLoft=std::clamp(spec_.loftRad,0.,.7);
            const double flightPitch=std::asin(std::clamp(dot(heading,up),-1.,1.));
            const double pitchFactor=std::clamp((targetLoft-flightPitch)/std::max(targetLoft,0.05),0.,1.);
            const double tgoFactor=std::isfinite(out.timeToGoS)?std::clamp((out.timeToGoS-6.)/8.,0.,1.):1.;
            const double loft=std::sin(targetLoft)*spec_.loftAccelerationG*9.80665*pitchFactor*tgoFactor;
            if(loft>0)steering=add(steering,scale(subtract(up,scale(heading,dot(up,heading))),loft));
        }
    }
    steering=subtract(steering,scale(heading,dot(steering,heading)));
    const auto command=add(steering,scale(unit(f.up),9.80665));
    const auto magnitude=norm(command),limit=spec_.maxG*9.80665;
    if(!finite(command)||!std::isfinite(magnitude)||!std::isfinite(limit))return {};
    out.acceleration=magnitude>limit&&magnitude>0?scale(command,limit/magnitude):command;
    terminalAcquiredV3_=terminal;
    return out;
}
}
