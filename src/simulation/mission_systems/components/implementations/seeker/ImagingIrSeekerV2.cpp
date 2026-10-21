#include "ImagingIrSeekerV2.h"
namespace fsim::ms::components {
using namespace measured;
ImagingIrSeekerV2::ImagingIrSeekerV2(weapon::Spec s):MeasuredSeekerV2(std::move(s)) {}
SeekerResultV2 ImagingIrSeekerV2::sample(const SeekerFrameV2& raw) {
    const auto admission=admit(raw);
    if(admission==Admission::Repeated)return cached_;
    if(admission==Admission::Invalid)return {};
    auto f=raw;for(auto& s:f.samples) {s.rangeM.reset();s.rangeRateMs.reset();}
    const auto candidates=eligible(f);auto chosen=nearest(f,candidates);
    bool spatial=chosen&&f.samples[*chosen].imageExtent.has_value();
    std::vector<std::size_t> gated;
    const auto cue=reference(f);
    for(auto i:candidates)if(dot(cue,unit(f.samples[i].direction))>=config().associationCosine) {
        gated.push_back(i);spatial=spatial&&f.samples[i].imageExtent.has_value();
    }
    if(chosen&&spatial&&track_&&cached_.tracking&&track_->imageExtent&&!gated.empty()) {
        const auto old=*track_->imageExtent;
        const auto oldWidth=old.widthRad/std::hypot(old.widthRad,old.heightRad);
        double residual=std::numeric_limits<double>::infinity(),angular=-2;
        for(auto i:gated) {
            const auto e=*f.samples[i].imageExtent;
            const auto error=std::abs(e.widthRad/std::hypot(e.widthRad,e.heightRad)-oldWidth);
            const auto alignment=dot(cue,unit(f.samples[i].direction));
            if(error<residual||(error==residual&&alignment>angular)) {
                chosen=i;residual=error;angular=alignment;
            }
        }
    }
    auto result=finish(f,chosen);
    // Ideal spatial/extent association, not pixel imagery or an IRCM classifier.
    // Missing competing extent data never becomes evidence of a flare.
    result.spatialFeaturesAvailable=spatial;cached_=result;return result;
}
}
