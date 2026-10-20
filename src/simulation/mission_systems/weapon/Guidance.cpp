#include "Guidance.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace fsim::ms::weapon {
namespace {
constexpr double gravity = 9.80665;
constexpr double cueCosine = 0.984807753012208; // cos(10 degrees).
V3 add(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 subtract(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 scale(V3 a, double factor) { return {a.x * factor, a.y * factor, a.z * factor}; }
double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double length(V3 a) { return std::hypot(a.x, a.y, a.z); }
V3 unit(V3 a) {
    const double norm = length(a);
    return norm > 1e-9 ? scale(a, 1 / norm) : V3{};
}
bool finite(V3 a) {
    return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}
bool valid(const Observation& observation) {
    return observation.id != 0 && finite(observation.position) && finite(observation.velocity)
        && std::isfinite(observation.signal) && observation.signal > 0
        && (std::isnan(observation.admittedRangeM)
            || (std::isfinite(observation.admittedRangeM) && observation.admittedRangeM >= 0));
}
bool valid(const Spec& spec) {
    return std::isfinite(spec.acquireM) && spec.acquireM >= 0
        && std::isfinite(spec.gimbalRad) && spec.gimbalRad >= 0 && spec.gimbalRad <= 3.141592653589793
        && std::isfinite(spec.maxG) && spec.maxG >= 0
        && std::isfinite(spec.lifeS) && spec.lifeS >= 0
        && std::isfinite(spec.armingS) && spec.armingS >= 0
        && std::isfinite(spec.fuzeM) && spec.fuzeM >= 0
        && std::isfinite(spec.loftRad) && spec.capabilityVersion <= 1;
}
V3 lateral(V3 acceleration, V3 velocityUnit) {
    return subtract(acceleration, scale(velocityUnit, dot(acceleration, velocityUnit)));
}
}

Guidance::Guidance(Spec spec) : spec_(std::move(spec)) {}
Guidance::Guidance(Spec spec, std::optional<Observation> initialEstimate) : spec_(std::move(spec)) {
    if (initialEstimate && valid(*initialEstimate)) estimate_ = std::move(initialEstimate);
}
Spec Guidance::spec() const noexcept { return spec_; }
Output Guidance::update(const Input& in) {
    Output out;
    const bool radar = spec_.seeker == "active_radar";
    const bool sarh = spec_.seeker == "semi_active_radar";
    const bool infrared = spec_.seeker.find("infrared") != std::string::npos;
    if ((!radar && !sarh && !infrared) || !valid(spec_) || !finite(in.position) || !finite(in.velocity)
        || !std::isfinite(in.ageS) || in.ageS < 0 || !std::isfinite(in.dtS) || in.dtS < 0
        || (initialized_ && in.ageS < lastAgeS_)) return out;
    double boresightNorm = 0;
    if (in.boresight) {
        if (!finite(*in.boresight)) return out;
        boresightNorm = std::hypot(in.boresight->x, in.boresight->y, in.boresight->z);
        if (!(boresightNorm > 0) || !std::isfinite(boresightNorm)) return out;
    }
    // Compute a complete candidate state before committing. Finite inputs can
    // still overflow extrapolation, PN, timing or command arithmetic; refusing
    // such an update must not advance the accepted clock or seeker state.
    bool nextActive = active_;
    std::uint32_t nextHolding = holding_;
    auto nextEstimate = estimate_;
    double nextEstimateAge = estimateAgeS_;
    const auto commit = [&]() {
        active_ = nextActive;
        holding_ = nextHolding;
        estimate_ = nextEstimate;
        estimateAgeS_ = nextEstimateAge;
        initialized_ = true;
        lastAgeS_ = in.ageS;
    };
    const bool acceptsUpdates = spec_.capabilityVersion == 0 || spec_.launcherUpdates;
    const bool inertialMidcourse = spec_.capabilityVersion == 0 ? radar : spec_.inertialMidcourse;
    out.supported = sarh ? in.illuminated : in.supported && acceptsUpdates;
    if (acceptsUpdates && in.supported && in.estimate && valid(*in.estimate)) {
        nextEstimate = in.estimate;
        nextEstimateAge = in.ageS;
    }
    std::optional<Observation> prediction = nextEstimate;
    if (prediction) {
        const auto movement = scale(prediction->velocity, std::max(0.0, in.ageS - nextEstimateAge));
        if (!finite(movement)) return Output{};
        prediction->position = add(prediction->position, movement);
        if (!finite(prediction->position)) return Output{};
    }

    const double acquisition = std::max(0.0, spec_.acquireM);
    const double activationRange = acquisition * 1.15;
    const double speed = length(in.velocity);
    const double maximum = spec_.maxG * gravity;
    if (!std::isfinite(activationRange) || !std::isfinite(speed) || !std::isfinite(maximum)) return Output{};
    if (prediction) {
        const auto relative = subtract(prediction->position, in.position);
        if (!finite(relative) || !std::isfinite(length(relative))) return Output{};
    }
    if (radar && (!prediction || length(subtract(prediction->position, in.position)) <= activationRange)) nextActive = true;
    out.active = radar && nextActive;
    if (out.active) out.timeToActiveS = 0;
    out.holding = nextHolding;
    const V3 heading = unit(in.velocity);
    const V3 seekerAxis = in.boresight ? V3{in.boresight->x / boresightNorm,
        in.boresight->y / boresightNorm, in.boresight->z / boresightNorm} : heading;
    const double gimbalCosine = std::cos(std::clamp(spec_.gimbalRad, 0.0, 3.141592653589793));
    auto eligible = [&](const Observation& observation) {
        if (!valid(observation)) return false;
        const V3 relative = subtract(observation.position, in.position);
        const double range = length(relative);
        if (!finite(relative) || !std::isfinite(range)) return false;
        const double admittedRange = std::isnan(observation.admittedRangeM)
            ? acquisition : observation.admittedRangeM;
        return range <= admittedRange && (range < 1e-9 || (!in.boresight && length(in.velocity) < 1e-9)
            || dot(unit(relative), seekerAxis) >= gimbalCosine);
    };

    const Observation* observed = nullptr;
    const bool canObserve = (radar && nextActive) || (sarh && in.illuminated) || infrared;
    const bool firstCue=!initialized_ && infrared && in.launchCue!=0;
    if (canObserve) {
        if (firstCue) for (const auto& observation : in.visible)
            if (observation.id==in.launchCue && eligible(observation)) { observed=&observation; break; }
        // A held contact remains held only while a current observation exists.
        for (const auto& observation : in.visible) if (!observed) {
            if (observation.id == nextHolding && eligible(observation)) {
                observed = &observation;
                break;
            }
        }
        if (observed) {
            // basis: assumed. Sensor strength discrimination in the current two-degree association cell.
            // Imaging seekers require a stronger distractor; no random outcome or target truth is consulted.
            const double resistance=spec_.seeker.find("imaging")!=std::string::npos?5.0:1.6;
            const Observation* strongest=observed;
            const V3 heldRay=unit(subtract(observed->position,in.position));
            for (const auto& alternative : in.visible) if (alternative.decoy && eligible(alternative) &&
                    alternative.signal / resistance > observed->signal &&
                    dot(heldRay,unit(subtract(alternative.position,in.position)))>.999390827019096 &&
                    (alternative.signal>strongest->signal || (alternative.signal==strongest->signal && alternative.id<strongest->id)))
                strongest=&alternative;
            observed=strongest;
        }
        if (!observed && !firstCue) {
            double bestDistance = std::numeric_limits<double>::infinity();
            for (const auto& observation : in.visible) {
                if (!eligible(observation)) continue;
                if (prediction) {
                    const V3 cue = subtract(prediction->position, in.position);
                    const V3 ray = subtract(observation.position, in.position);
                    if (length(cue) > 1e-9 && length(ray) > 1e-9
                        && dot(unit(cue), unit(ray)) < cueCosine) continue;
                }
                const double distance = length(subtract(observation.position,
                    prediction ? prediction->position : in.position));
                if (!std::isfinite(distance)) continue;
                if (distance < bestDistance || (distance == bestDistance
                    && observed && observation.id < observed->id)) {
                    bestDistance = distance;
                    observed = &observation;
                }
            }
        }
    }
    if (observed) {
        nextHolding = observed->id;
        out.holding = nextHolding;
        out.tracking = true;
        out.phase = WeaponPhase::Homing;
        nextEstimate = *observed;
        nextEstimateAge = in.ageS;
        prediction = *observed;
    } else if ((radar && !nextActive) || (!radar && inertialMidcourse && !nextHolding && prediction)) {
        out.phase = WeaponPhase::Midcourse;
    } else {
        out.phase = nextHolding ? WeaponPhase::Lost : WeaponPhase::Seeking;
    }

    if (!prediction) { commit(); return out; }
    const V3 relative = subtract(prediction->position, in.position);
    const V3 relativeVelocity = subtract(prediction->velocity, in.velocity);
    const double range = length(relative);
    if (!finite(relative) || !finite(relativeVelocity) || !std::isfinite(range)) return Output{};
    const double closing = range > 1e-9 ? -dot(relativeVelocity, unit(relative)) : 0;
    if (!std::isfinite(closing)) return Output{};
    if (closing > 1e-9) {
        out.timeToGoS = range / closing;
        if (!std::isfinite(out.timeToGoS)) return Output{};
        if (radar && !nextActive) {
            out.timeToActiveS = std::max(0.0, range - activationRange) / closing;
            if (!std::isfinite(out.timeToActiveS)) return Output{};
        }
    }
    // Active search does not revoke permitted midcourse steering. Sensor
    // activation/observation and acceleration execution are separate: an ARH
    // estimate can steer without authorizing a seeker hold or a fuze.
    // A declaration governs pre-acquisition inertial control. Once the seeker
    // has actually held an observation, its existing Lost memory remains an
    // onboard source, independently of launcher correction capability.
    if (!inertialMidcourse && (out.phase == WeaponPhase::Seeking
        || (spec_.capabilityVersion == 1 && !nextHolding && !out.tracking))) { commit(); return out; }
    V3 command{};
    if (range > 1e-9 && closing > 0 && speed > 1e-9) {
        const V3 numerator = cross(relative, relativeVelocity);
        const double gain = 3.5 * closing;
        if (!finite(numerator) || !std::isfinite(gain)) return Output{};
        const V3 lineRate = scale(numerator, (1 / range) / range);
        const V3 steering = scale(cross(lineRate, heading), gain);
        if (!finite(lineRate) || !finite(steering)) return Output{};
        command = lateral(steering, heading);
        if (!finite(command)) return Output{};
    }
    if (finite(in.up) && !std::isfinite(length(in.up))) return Output{};
    const V3 up = finite(in.up) ? unit(in.up) : V3{};
    if (out.phase == WeaponPhase::Midcourse) {
        const double loft = std::sin(std::clamp(spec_.loftRad, 0.0, 0.7)) * 2 * gravity;
        command = add(command, lateral(scale(up, loft), heading));
    }
    command = add(command, scale(up, gravity));
    const double magnitude = length(command);
    if (!finite(command) || !std::isfinite(magnitude)) return Output{};
    if (magnitude > maximum && magnitude > 1e-9) command = scale(command, maximum / magnitude);
    out.acceleration = command;
    commit();
    return out;
}

double sweptMissDistance(V3 ownBefore, V3 ownAfter, V3 targetBefore,
                         V3 targetAfter, double* fraction) noexcept {
    const V3 before = subtract(ownBefore, targetBefore);
    const V3 movement = subtract(subtract(ownAfter, targetAfter), before);
    const double square = dot(movement, movement);
    const double t = square > 1e-18 ? std::clamp(-dot(before, movement) / square, 0.0, 1.0) : 0;
    if (fraction) *fraction = t;
    return length(add(before, scale(movement, t)));
}
bool armed(double ageS, const Spec& spec) noexcept {
    return valid(spec) && std::isfinite(ageS) && ageS >= 0 && ageS >= spec.armingS;
}
}
