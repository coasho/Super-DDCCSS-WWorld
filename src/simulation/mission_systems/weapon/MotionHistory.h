#pragma once

#include "Fuze.h"
#include <vector>

namespace fsim::ms::weapon {

// Offsets from the current World step, not model-global timestamps. Captured
// native samples define piecewise linear sweeps; no new motion is integrated here.
struct MotionSample { double offsetS = 0; V3 position; };

inline bool validMotionPath(const std::vector<MotionSample>& path) noexcept {
    if (path.empty() || path.front().offsetS != 0) return false;
    double previous=-1;
    for (const auto& sample : path) {
        if (!std::isfinite(sample.offsetS) || sample.offsetS <= previous ||
            !fuze_detail::finite(sample.position)) return false;
        previous=sample.offsetS;
    }
    return true;
}

// A body stopped by native contact remains at its last pose. A weapon's own
// eligible interval is separately truncated at contact, before arming/minimizing.
inline V3 motionAt(const std::vector<MotionSample>& path, double offsetS) noexcept {
    for (std::size_t k=1;k<path.size();++k) if (offsetS <= path[k].offsetS) {
        const auto& a=path[k-1]; const auto& b=path[k];
        return fuze_detail::at(a.position,fuze_detail::subtract(b.position,a.position),
                              (offsetS-a.offsetS)/(b.offsetS-a.offsetS));
    }
    return path.back().position;
}

inline FuzeResult sweptPathFuze(const std::vector<MotionSample>& own,
                               const std::vector<MotionSample>& target,
                               double ageBeforeS, double stepDurationS, const Spec& spec,
                               double eligibleEndS, bool exclusiveEnd) {
    FuzeResult result;
    if (!fuze_detail::validModel(spec) || !validMotionPath(own) || !validMotionPath(target) ||
        !std::isfinite(ageBeforeS) || ageBeforeS < 0 ||
        !std::isfinite(stepDurationS) || stepDurationS <= 0 ||
        !std::isfinite(eligibleEndS) || eligibleEndS < 0 || eligibleEndS > stepDurationS ||
        !std::isfinite(spec.armingS) || spec.armingS<0 ||
        !std::isfinite(spec.lifeS) || spec.lifeS<=spec.armingS ||
        !std::isfinite(spec.fuzeM) || spec.fuzeM<0) return result;
    result.valid=true;
    const double end=spec.fuzeModelVersion==1
        ? std::min({eligibleEndS,own.back().offsetS,target.back().offsetS})
        : std::min(eligibleEndS,own.back().offsetS);
    if (end<=0) return result;
    auto clipped=spec;
    if (exclusiveEnd) clipped.lifeS=std::min(spec.lifeS,ageBeforeS+end);
    if (clipped.lifeS<=clipped.armingS) return result;
    std::vector<double> times{0,end};
    for (const auto* path : {&own,&target}) for (const auto& sample : *path)
        if (sample.offsetS>0 && sample.offsetS<end) times.push_back(sample.offsetS);
    std::sort(times.begin(),times.end());
    times.erase(std::unique(times.begin(),times.end()),times.end());
    std::vector<FuzeResult> parts;
    if (spec.fuzeModelVersion==1) parts.reserve(times.size()-1);
    for (std::size_t k=1;k<times.size();++k) {
        const double first=times[k-1],last=times[k];
        const auto part=sweptFuze(motionAt(own,first),motionAt(own,last),
                                 motionAt(target,first),motionAt(target,last),
                                 ageBeforeS+first,ageBeforeS+last,clipped);
        if (!part.valid) return FuzeResult{};
        if (spec.fuzeModelVersion==1) parts.push_back(part);
        if (!std::isfinite(part.missM)) continue;
        const double fraction=(first+part.fraction*(last-first))/stepDurationS;
        // Native samples partition the same captured eligible interval. Select
        // its closest approach, rather than a sampling-dependent first segment
        // endpoint inside the radius. Keep the earliest time for an exact tie.
        if (!std::isfinite(result.missM) || part.missM<result.missM)
            result={true,part.triggered,part.missM,fraction};
    }
    if (spec.fuzeModelVersion==0) return result;

    // Explicit V1 does not initiate at the interval's constrained minimum.
    // Preserve that geometry for referee diagnostics, then independently find
    // the first actual captured stationary point or closing-to-opening corner.
    result.triggered=false;
    const auto relativeAt=[&](double t) {
        return fuze_detail::subtract(motionAt(own,t),motionAt(target,t));
    };
    const auto candidate=[&](double t,V3 relative) {
        FuzeResult found;
        const double age=ageBeforeS+t;
        const double miss=std::hypot(relative.x,relative.y,relative.z);
        if (std::isfinite(age) && age>=spec.armingS && age<clipped.lifeS &&
            std::isfinite(miss) && miss<=spec.fuzeM)
            found={true,true,miss,t/stepDurationS};
        return found;
    };
    for (std::size_t k=1;k<times.size();++k) {
        const auto& part=parts[k-1];
        if (part.triggered && part.fraction>0 && part.fraction<1) {
            const double t=times[k-1]+part.fraction*(times[k]-times[k-1]);
            const auto found=candidate(t,relativeAt(t));
            if (found.triggered) return found;
        }
        const double t=times[k];
        const auto relative=relativeAt(t);
        if (k+1==times.size()) {
            // An analytic stationary point at the last captured endpoint is
            // represented, whereas a future point clamped to it is not.
            if (part.triggered && part.fraction==1) {
                const auto found=candidate(t,relative);
                if (found.triggered) return found;
            }
            continue;
        }
        const auto left=fuze_detail::subtract(relative,relativeAt(times[k-1]));
        const auto right=fuze_detail::subtract(relativeAt(times[k+1]),relative);
        const double before=fuze_detail::dot(relative,left);
        const double after=fuze_detail::dot(relative,right);
        const double moving=fuze_detail::dot(left,left);
        if (!std::isfinite(before) || !std::isfinite(after) || !std::isfinite(moving)) return {};
        if (before<=0 && after>=0 && (before<0 || moving>0)) {
            const auto found=candidate(t,relative);
            if (found.triggered) return found;
        }
    }
    return result;
}

/// Actual last captured relative segment from this release/target only. It
/// confirms a corner at a shared decision boundary after the opening segment
/// is captured. Unknown target, changed target, time gap, mismatched boundary
/// positions or reset discards prior history; no gap is interpolated. This
/// transient history is independent of a scheduled initiation in FuzeDelay.h.
class FuzeTurnStateV1 {
public:
    void reset() noexcept { *this=FuzeTurnStateV1{}; }
    FuzeResult evaluate(const std::vector<MotionSample>& own,
                        const std::vector<MotionSample>& target,
                        double ageBeforeS,double durationS,const Spec& spec,
                        double eligibleEndS,bool exclusiveEnd,std::uint32_t targetId) {
        auto result=sweptPathFuze(own,target,ageBeforeS,durationS,spec,eligibleEndS,exclusiveEnd);
        if (!result.valid || spec.fuzeModelVersion!=1 || targetId==0) {
            reset();
            return targetId==0 && spec.fuzeModelVersion==1 ? FuzeResult{} : result;
        }
        const double end=std::min({eligibleEndS,own.back().offsetS,target.back().offsetS});
        const bool contiguous=available_ && targetId==targetId_ && ageBeforeS==ageS_ &&
                              same(own.front().position,own_) && same(target.front().position,target_);
        if (contiguous && !reported_ && end>0 && ageBeforeS>=spec.armingS && ageBeforeS<spec.lifeS) {
            double first=end;
            for (const auto* path : {&own,&target})
                for (const auto& sample : *path) if (sample.offsetS>0) {first=std::min(first,sample.offsetS);break;}
            const auto relative=fuze_detail::subtract(own.front().position,target.front().position);
            const auto next=fuze_detail::subtract(motionAt(own,first),motionAt(target,first));
            const auto right=fuze_detail::subtract(next,relative);
            const double before=fuze_detail::dot(relative,movement_);
            const double after=fuze_detail::dot(relative,right);
            const double moving=fuze_detail::dot(movement_,movement_);
            const double miss=std::hypot(relative.x,relative.y,relative.z);
            if (!std::isfinite(before) || !std::isfinite(after) || !std::isfinite(moving) || !std::isfinite(miss)) {
                reset(); return {};
            }
            if (before<=0 && after>=0 && (before<0 || moving>0) && miss<=spec.fuzeM)
                result={true,true,miss,0};
        }
        reset();
        const double ageAfterS=ageBeforeS+durationS;
        if (!exclusiveEnd && end==durationS && std::isfinite(ageAfterS) && ageAfterS<spec.lifeS) {
            double previous=0;
            for (const auto* path : {&own,&target}) for (const auto& sample : *path)
                if (sample.offsetS<end) previous=std::max(previous,sample.offsetS);
            own_=motionAt(own,end); target_=motionAt(target,end);
            const auto relative=fuze_detail::subtract(own_,target_);
            const auto before=fuze_detail::subtract(motionAt(own,previous),motionAt(target,previous));
            movement_=fuze_detail::subtract(relative,before);
            if (!fuze_detail::finite(movement_)) {reset();return {};}
            available_=true; targetId_=targetId; ageS_=ageAfterS;
            reported_=result.triggered && result.fraction==1;
        }
        return result;
    }

private:
    static bool same(V3 a,V3 b) noexcept { return a.x==b.x && a.y==b.y && a.z==b.z; }
    bool available_=false,reported_=false;
    std::uint32_t targetId_=0;
    double ageS_=0;
    V3 own_,target_,movement_;
};

} // namespace fsim::ms::weapon
