#include "NonImagingIrSeekerV2.h"
namespace fsim::ms::components {
using namespace measured;
NonImagingIrSeekerV2::NonImagingIrSeekerV2(weapon::Spec s):MeasuredSeekerV2(std::move(s)) {}
SeekerResultV2 NonImagingIrSeekerV2::sample(const SeekerFrameV2& raw) {
    const auto admission=admit(raw);
    if(admission==Admission::Repeated)return cached_;
    if(admission==Admission::Invalid)return {};
    auto f=raw;
    for(auto& s:f.samples) {s.rangeM.reset();s.rangeRateMs.reset();s.imageExtent.reset();}
    const auto candidates=eligible(f);const auto anchor=nearest(f,candidates);
    if(!anchor)return finish(f,std::nullopt);
    const auto direction=unit(f.samples[*anchor].direction);
    double maximum=0;
    for(auto i:candidates)if(dot(direction,unit(f.samples[i].direction))>=config().associationCosine)
        maximum=std::max(maximum,f.samples[i].signal);
    V sum{};auto dominant=*anchor;
    for(auto i:candidates)if(dot(direction,unit(f.samples[i].direction))>=config().associationCosine) {
        sum=add(sum,scale(unit(f.samples[i].direction),f.samples[i].signal/maximum));
        if(f.samples[i].signal>f.samples[dominant].signal)dominant=i;
    }
    // Reduced-order unresolved angular cell: intensity centroid, no reticle or
    // hardware rejection claim. Referee attribution does not drive this ray.
    return finish(f,dominant,sum);
}
}
