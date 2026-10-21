#include "MeasuredSeekerV2.h"
#include <algorithm>
#include <cmath>
#include <tuple>

namespace fsim::ms::components {
namespace measured {
bool finite(V a) noexcept {return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
double norm(V a) noexcept {return std::hypot(a.x,a.y,a.z);}
double dot(V a,V b) noexcept {return a.x*b.x+a.y*b.y+a.z*b.z;}
V add(V a,V b) noexcept {return {a.x+b.x,a.y+b.y,a.z+b.z};}
V subtract(V a,V b) noexcept {return {a.x-b.x,a.y-b.y,a.z-b.z};}
V scale(V a,double b) noexcept {return {a.x*b,a.y*b,a.z*b};}
V cross(V a,V b) noexcept {return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
V unit(V a) noexcept {const auto n=norm(a);return finite(a)&&std::isfinite(n)&&n>0?V{a.x/n,a.y/n,a.z/n}:V{};}
bool validExtent(const ImageExtentV2& e) noexcept {
    return std::isfinite(e.widthRad)&&std::isfinite(e.heightRad)&&e.widthRad>0&&e.heightRad>0;
}
}
using namespace measured;

MeasuredSeekerV2::MeasuredSeekerV2(weapon::Spec s):spec_(std::move(s)) {}
const weapon::SeekerConfigV1& MeasuredSeekerV2::config() const noexcept {return spec_.seekerModel;}
double MeasuredSeekerV2::infraredRange(double a,double b,bool c) const noexcept {return weapon::infraredAcquisitionRangeM(a,b,c,config());}
double MeasuredSeekerV2::radarRange(double a,double b,bool c) const noexcept {return weapon::radarAcquisitionRangeM(a,b,c,config());}
bool MeasuredSeekerV2::notched(double a,double b) const noexcept {return weapon::radarNotched(a,b,config());}
double MeasuredSeekerV2::decoySignal(double a,bool b) const noexcept {return weapon::decoySignal(a,b,config());}
bool MeasuredSeekerV2::illuminated(const double a[3],double b,double c,double d,double e) const noexcept {
    return weapon::withinIlluminationBeam(a,b,c,d,e,config());
}
MeasuredSeekerV2::Admission MeasuredSeekerV2::admit(const SeekerFrameV2& f) const noexcept {
    if(!std::isfinite(f.timeS)||f.timeS<0||!std::isfinite(f.ageS)||f.ageS<0||
       !std::isfinite(f.dtS)||f.dtS<0||!finite(f.position)||!finite(f.velocity)||
       !finite(f.up)||!finite(f.boresight)||!(norm(f.boresight)>0)||
       !std::isfinite(norm(f.boresight))||f.timeS<lastSampleS_||
       (track_&&f.timeS<track_->sampleTimeS))return Admission::Invalid;
    return f.timeS==lastSampleS_?Admission::Repeated:Admission::New;
}
void MeasuredSeekerV2::resetTracking() noexcept {
    track_.reset();cached_={};lastSampleS_=std::numeric_limits<double>::quiet_NaN();nextTrack_=1;
}
void MeasuredSeekerV2::seedTrack(const SeekerTrackV2& t) {
    if(!t.localId||!std::isfinite(t.sampleTimeS)||!finite(t.direction)||!(norm(t.direction)>0)||
       (t.rateAvailable&&!finite(t.directionRate)))return;
    track_=t;track_->direction=unit(t.direction);cached_={};cached_.tracking=true;
    nextTrack_=t.localId==std::numeric_limits<std::uint32_t>::max()?1:t.localId+1;
}
std::vector<std::size_t> MeasuredSeekerV2::eligible(const SeekerFrameV2& f) const {
    std::vector<std::size_t> out;
    const auto axis=unit(f.boresight);
    for(std::size_t i=0;i<f.samples.size();++i) {
        const auto& s=f.samples[i];const auto n=norm(s.direction);
        if(!finite(s.direction)||!(n>0)||!std::isfinite(n)||!std::isfinite(s.signal)||!(s.signal>0)||
           dot(unit(s.direction),axis)<std::cos(config().gimbalRad))continue;
        if(s.rangeM&&(!std::isfinite(*s.rangeM)||*s.rangeM<config().minimumRangeM))continue;
        if(s.rangeRateMs&&!std::isfinite(*s.rangeRateMs))continue;
        if(s.imageExtent&&!validExtent(*s.imageExtent))continue;
        out.push_back(i);
    }
    // Observable ordering fixes arithmetic/ties independently of scene identity.
    const auto key=[](const SeekerSampleV2& s) {
        const auto direction=unit(s.direction);
        return std::tuple{direction.x,direction.y,direction.z,s.signal,s.rangeM,s.rangeRateMs,
            s.imageExtent.has_value(),s.imageExtent?s.imageExtent->widthRad:0.,
            s.imageExtent?s.imageExtent->heightRad:0.,s.illuminator,s.transmitterPathClear,s.receiverPathClear};
    };
    std::sort(out.begin(),out.end(),[&](auto a,auto b) {
        return key(f.samples[a])<key(f.samples[b]);
    });
    return out;
}
weapon::V3 MeasuredSeekerV2::reference(const SeekerFrameV2& f) const noexcept {
    if(track_&&cached_.tracking) {
        const auto predicted=track_->rateAvailable?
            add(track_->direction,scale(track_->directionRate,std::max(0.,f.timeS-track_->sampleTimeS))):track_->direction;
        if(finite(predicted)&&norm(predicted)>0)return unit(predicted);
    }
    return unit(f.boresight);
}
std::optional<std::size_t> MeasuredSeekerV2::nearest(const SeekerFrameV2& f,const std::vector<std::size_t>& ids) const {
    const auto cue=reference(f);std::optional<std::size_t> best;double cosine=-2;
    for(const auto i:ids) {
        const auto value=dot(cue,unit(f.samples[i].direction));
        if(track_&&cached_.tracking&&value<config().associationCosine)continue;
        if(value>cosine) {cosine=value;best=i;}
    }
    return best;
}
SeekerResultV2 MeasuredSeekerV2::finish(const SeekerFrameV2& f,std::optional<std::size_t> selected,
                                       std::optional<weapon::V3> aggregate) {
    SeekerResultV2 next;next.valid=true;next.supported=f.supported;next.track=track_;
    if(selected) {
        const auto& s=f.samples[*selected];SeekerTrackV2 t;
        t.direction=unit(aggregate.value_or(s.direction));t.sampleTimeS=f.timeS;
        if(!finite(t.direction)||!(norm(t.direction)>0))return {};
        t.localId=track_&&cached_.tracking?track_->localId:nextTrack_;
        if(track_&&cached_.tracking) {
            const auto dt=f.timeS-track_->sampleTimeS;
            if(dt>0) {
                const auto derivative=scale(subtract(t.direction,track_->direction),1/dt);
                t.directionRate=subtract(derivative,scale(t.direction,dot(derivative,t.direction)));
                t.rateAvailable=finite(t.directionRate);
                if(!t.rateAvailable)return {};
            } else {t.directionRate=track_->directionRate;t.rateAvailable=track_->rateAvailable;}
        }
        t.rangeM=s.rangeM;t.rangeRateMs=s.rangeRateMs;t.imageExtent=s.imageExtent;
        if(!track_||!cached_.tracking)nextTrack_=nextTrack_==std::numeric_limits<std::uint32_t>::max()?1:nextTrack_+1;
        track_=t;next.track=t;next.tracking=true;next.attributionIndex=selected;
    }
    lastSampleS_=f.timeS;cached_=next;return cached_;
}
}
