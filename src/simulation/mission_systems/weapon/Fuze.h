#pragma once

#include "Guidance.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::ms::weapon {

struct FuzeResult {
    bool valid = false;
    bool triggered = false;
    double missM = std::numeric_limits<double>::quiet_NaN();
    double fraction = std::numeric_limits<double>::quiet_NaN();
};

namespace fuze_detail {
inline bool validModel(const Spec& spec) noexcept {
    if (spec.fuzeModelVersion==0) return spec.fuzeAlgorithm==0;
    return spec.fuzeModelVersion==1 && spec.fuzeAlgorithm==1 &&
           std::isfinite(spec.fuzeDelayS) && spec.fuzeDelayS>=0;
}
inline bool finite(V3 v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
inline V3 subtract(V3 a, V3 b) noexcept { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline double dot(V3 a, V3 b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 at(V3 before, V3 movement, double fraction) noexcept {
    return {std::fma(movement.x, fraction, before.x),
            std::fma(movement.y, fraction, before.y),
            std::fma(movement.z, fraction, before.z)};
}
}

// Linear relative-motion sweep restricted to the armed, live ages [armingS,
// lifeS). The outer step includes both supplied endpoints. fraction is in the
// ORIGINAL step. Model 0 preserves the legacy interval minimum (including its
// clamped endpoints). Explicit model 1/law 1 initiates only at an actual linear
// stationary point captured in the active interval, never at a clamped approach
// endpoint. A stationary relative overlap alone establishes no V1 turn. Path
// corners and retained cross-decision history belong to MotionHistory.h.
// Legacy expiry may retain strict interior overlap; V1 never nudges an excluded
// stationary point into a live initiation.
//
// valid=false rejects malformed inputs or overflowing geometry. A valid step
// with no active interval returns no geometry (NaNs). The caller still owns
// armed-release/current-observation permission, damage and terminal events.
inline FuzeResult sweptFuze(V3 ownBefore, V3 ownAfter, V3 targetBefore, V3 targetAfter,
                            double ageBeforeS, double ageAfterS, const Spec& spec) noexcept {
    FuzeResult result;
    if (!fuze_detail::validModel(spec)
        || !fuze_detail::finite(ownBefore) || !fuze_detail::finite(ownAfter)
        || !fuze_detail::finite(targetBefore) || !fuze_detail::finite(targetAfter)
        || !std::isfinite(ageBeforeS) || !std::isfinite(ageAfterS)
        || ageBeforeS < 0 || ageAfterS <= ageBeforeS
        || !std::isfinite(spec.fuzeM) || spec.fuzeM < 0
        || !std::isfinite(spec.armingS) || spec.armingS < 0
        || !std::isfinite(spec.lifeS) || spec.lifeS <= spec.armingS) return result;

    const double duration = ageAfterS - ageBeforeS;
    const V3 before = fuze_detail::subtract(ownBefore, targetBefore);
    const V3 after = fuze_detail::subtract(ownAfter, targetAfter);
    const V3 movement = fuze_detail::subtract(after, before);
    const double square = fuze_detail::dot(movement, movement);
    const double projection = fuze_detail::dot(before, movement);
    if (!std::isfinite(duration) || !fuze_detail::finite(before) || !fuze_detail::finite(after)
        || !fuze_detail::finite(movement) || !std::isfinite(square) || !std::isfinite(projection)) return result;

    result.valid = true;
    const double startAge = std::max(ageBeforeS, spec.armingS);
    const double endAge = std::min(ageAfterS, spec.lifeS);
    if (startAge > endAge || startAge >= spec.lifeS) return result;
    double first = std::clamp((startAge - ageBeforeS) / duration, 0.0, 1.0);
    const double last = std::clamp((endAge - ageBeforeS) / duration, 0.0, 1.0);
    const bool expiryEnd = endAge == spec.lifeS;
    const auto ageAt = [&](double fraction) { return ageBeforeS + fraction * duration; };
    // A rounded lower bound must not select a time before inclusive arming.
    if (ageAt(first) < spec.armingS) first = std::nextafter(first, 1.0);
    if (first > last || (expiryEnd && first == last) || ageAt(first) >= spec.lifeS) return result;

    const double unconstrained = square > 0 ? -projection / square : first;
    if (!std::isfinite(unconstrained)) return FuzeResult{};
    const bool actualCpa=square>0 && unconstrained>=first && unconstrained<=last &&
                         (!expiryEnd || unconstrained<last) &&
                         ageAt(unconstrained)>=spec.armingS && ageAt(unconstrained)<spec.lifeS;
    double closest = std::clamp(unconstrained, first, last);
    const V3 endpoint = fuze_detail::at(before, movement, closest);
    const double endpointMiss = std::hypot(endpoint.x, endpoint.y, endpoint.z);
    if (!fuze_detail::finite(endpoint) || !std::isfinite(endpointMiss)) return FuzeResult{};

    // A radius tangent reached only at lifeS is outside the active interval.
    // Decide that before nudging its fraction; rounding must not admit it.
    const bool excludedContact = expiryEnd && closest == last && endpointMiss >= spec.fuzeM;
    if (expiryEnd && (closest == last || ageAt(closest) >= spec.lifeS)) {
        const double liveAge = std::nextafter(spec.lifeS, ageBeforeS);
        closest = std::min(closest, std::clamp((liveAge - ageBeforeS) / duration, first, last));
        if (closest >= last) closest = std::nextafter(closest, first);
        if (ageAt(closest) >= spec.lifeS) closest = std::nextafter(closest, first);
        if (ageAt(closest) >= spec.lifeS) closest = first;
    }
    if (closest < first || closest > last || ageAt(closest) < spec.armingS || ageAt(closest) >= spec.lifeS)
        return result;
    const V3 relative = fuze_detail::at(before, movement, closest);
    const double miss = std::hypot(relative.x, relative.y, relative.z);
    if (!fuze_detail::finite(relative) || !std::isfinite(miss)) return FuzeResult{};
    result.missM = miss;
    result.fraction = closest;
    result.triggered = !excludedContact && miss <= spec.fuzeM &&
                       (spec.fuzeModelVersion==0 || actualCpa);
    return result;
}

} // namespace fsim::ms::weapon
