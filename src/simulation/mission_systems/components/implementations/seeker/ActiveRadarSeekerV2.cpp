#include "ActiveRadarSeekerV2.h"
namespace fsim::ms::components {
using namespace measured;
ActiveRadarSeekerV2::ActiveRadarSeekerV2(weapon::Spec s,std::optional<weapon::Observation> initial)
    :MeasuredSeekerV2(std::move(s)) {
    if(initial&&finite(initial->position)&&finite(initial->velocity))
        estimate_=LauncherEstimateV2{initial->position,initial->velocity,0};
}
void ActiveRadarSeekerV2::resetTracking() noexcept {
    MeasuredSeekerV2::resetTracking();
    active_=false;initialEpoch_=true;estimate_.reset();
}
ActiveRadarSeekerV2::PhasePreviewV1 ActiveRadarSeekerV2::previewPhaseV1(const SeekerFrameV2& f) const {
    auto estimate=estimate_;
    if(estimate&&initialEpoch_)estimate->sampleTimeS=f.timeS-f.ageS;
    if(spec_.launcherUpdates&&f.supported&&f.launcherEstimate) {
        const auto& e=*f.launcherEstimate;
        if(!std::isfinite(e.sampleTimeS)||e.sampleTimeS>f.timeS||!finite(e.position)||!finite(e.velocity))return {};
        if(!estimate||e.sampleTimeS>=estimate->sampleTimeS)estimate=e;
    }
    std::optional<LauncherEstimateV2> prediction=estimate;
    if(prediction) {
        prediction->position=add(prediction->position,scale(prediction->velocity,f.timeS-prediction->sampleTimeS));
        prediction->sampleTimeS=f.timeS;
        if(!finite(prediction->position))return {};
    }
    const auto activationRange=spec_.acquireM*spec_.activationRatio;
    if(!std::isfinite(activationRange))return {};
    const auto range=prediction?norm(subtract(prediction->position,f.position)):0;
    if(!std::isfinite(range))return {};
    const bool active=active_||!prediction||range<=activationRange;
    return {true,active,range,activationRange,estimate,prediction};
}
SeekerResultV2 ActiveRadarSeekerV2::sample(const SeekerFrameV2& f) {
    const auto admission=admit(f);
    if(admission==Admission::Repeated)return cached_;
    if(admission==Admission::Invalid)return {};
    const auto phase=previewPhaseV1(f);if(!phase.valid)return {};
    const auto& estimate=phase.estimate;const auto& prediction=phase.prediction;
    const auto range=phase.rangeM,activationRange=phase.activationRangeM;const bool active=phase.active;
    auto candidates=active?eligible(f):std::vector<std::size_t>{};
    auto selected=nearest(f,candidates);
    if(prediction&&(!track_||!cached_.tracking)&&active) {
        selected.reset();double best=std::cos(spec_.cueRad);
        const auto cue=unit(subtract(prediction->position,f.position));
        for(auto i:candidates) {
            const auto alignment=dot(cue,unit(f.samples[i].direction));
            if(alignment>=best) {selected=i;best=alignment;}
        }
    }
    auto result=finish(f,selected);if(!result.valid)return {};
    active_=active;estimate_=estimate;initialEpoch_=false;
    result.active=active;
    if(spec_.inertialMidcourse)result.midcourseEstimate=prediction;
    if(active)result.timeToActiveS=0;
    else if(prediction&&range>0) {
        const auto closing=-dot(subtract(prediction->velocity,f.velocity),unit(subtract(prediction->position,f.position)));
        if(closing>0&&std::isfinite(closing))result.timeToActiveS=std::max(0.,range-activationRange)/closing;
    }
    cached_=result;return result;
}
}
