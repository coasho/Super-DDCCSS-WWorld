#include "SemiActiveRadarSeekerV2.h"
namespace fsim::ms::components {
SemiActiveRadarSeekerV2::SemiActiveRadarSeekerV2(weapon::Spec s):MeasuredSeekerV2(std::move(s)) {}
SeekerResultV2 SemiActiveRadarSeekerV2::sample(const SeekerFrameV2& f) {
    const auto admission=admit(f);
    if(admission==Admission::Repeated)return cached_;
    if(admission==Admission::Invalid)return {};
    auto receiver=f;
    // No independent target-range channel is admitted for this receiver model.
    for(auto& s:receiver.samples) {s.rangeM.reset();s.rangeRateMs.reset();s.imageExtent.reset();}
    auto candidates=eligible(receiver);
    candidates.erase(std::remove_if(candidates.begin(),candidates.end(),[&](auto i) {
        const auto& s=receiver.samples[i];
        return !f.illuminated||!f.configuredIlluminator||s.illuminator!=f.configuredIlluminator||
               !s.transmitterPathClear||!s.receiverPathClear;
    }),candidates.end());
    auto result=finish(receiver,nearest(receiver,candidates));
    result.supported=f.illuminated&&f.configuredIlluminator!=0;cached_=result;return result;
}
}
