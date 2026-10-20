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
    if (!validMotionPath(own) || !validMotionPath(target) ||
        !std::isfinite(ageBeforeS) || ageBeforeS < 0 ||
        !std::isfinite(stepDurationS) || stepDurationS <= 0 ||
        !std::isfinite(eligibleEndS) || eligibleEndS < 0 || eligibleEndS > stepDurationS ||
        !std::isfinite(spec.armingS) || spec.armingS<0 ||
        !std::isfinite(spec.lifeS) || spec.lifeS<=spec.armingS ||
        !std::isfinite(spec.fuzeM) || spec.fuzeM<0) return result;
    result.valid=true;
    const double end=std::min(eligibleEndS,own.back().offsetS);
    if (end<=0) return result;
    auto clipped=spec;
    if (exclusiveEnd) clipped.lifeS=std::min(spec.lifeS,ageBeforeS+end);
    if (clipped.lifeS<=clipped.armingS) return result;
    std::vector<double> times{0,end};
    for (const auto* path : {&own,&target}) for (const auto& sample : *path)
        if (sample.offsetS>0 && sample.offsetS<end) times.push_back(sample.offsetS);
    std::sort(times.begin(),times.end());
    times.erase(std::unique(times.begin(),times.end()),times.end());
    for (std::size_t k=1;k<times.size();++k) {
        const double first=times[k-1],last=times[k];
        const auto part=sweptFuze(motionAt(own,first),motionAt(own,last),
                                 motionAt(target,first),motionAt(target,last),
                                 ageBeforeS+first,ageBeforeS+last,clipped);
        if (!part.valid) return FuzeResult{};
        if (!std::isfinite(part.missM)) continue;
        const double fraction=(first+part.fraction*(last-first))/stepDurationS;
        // Preserve the earliest triggering segment; minimum non-triggering miss
        // is diagnostic only and cannot override a prior physical encounter.
        if (part.triggered) return {true,true,part.missM,fraction};
        if (!std::isfinite(result.missM) || part.missM<result.missM)
            result={true,false,part.missM,fraction};
    }
    return result;
}

} // namespace fsim::ms::weapon
