#include "Guidance.h"
#include "simulation/mission_systems/Cards.h"
#include "simulation/mission_systems/components/Profile.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <iterator>
#include <utility>
#include <stdexcept>

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
        && std::isfinite(spec.loftRad) && spec.capabilityVersion <= 1
        && validSeekerConfigV1(spec.seekerModel)
        && (spec.seekerModel.version==0 || (spec.seeker==seekerModeNameV1(spec.seekerModel.mode) &&
            spec.acquireM==spec.seekerModel.acquireM && spec.gimbalRad==spec.seekerModel.gimbalRad))
        && spec.executionVersion <= 1
        && (spec.executionVersion == 0 || (spec.guidanceAlgorithm == 1
            && std::isfinite(spec.navigationGain) && spec.navigationGain > 2
            && std::isfinite(spec.cueRad) && spec.cueRad >= 0 && spec.cueRad <= 3.141592653589793
            && std::isfinite(spec.activationRatio) && spec.activationRatio > 0
            && spec.loftRad >= 0 && spec.loftRad <= .7
            && std::isfinite(spec.loftAccelerationG) && spec.loftAccelerationG >= 0));
}
} // namespace

bool configureSeeker(const components::DescriptorV1& descriptor, Spec& spec, std::string& error) {
    SeekerConfigV1 configured;
    if(!configureSeeker(descriptor,configured,error)) return false;
    auto candidate=spec;
    candidate.seekerModel=configured;candidate.seeker=seekerModeNameV1(configured.mode);
    candidate.acquireM=configured.acquireM;candidate.gimbalRad=configured.gimbalRad;
    spec=std::move(candidate);error.clear();return true;
}
bool configureGuidance(const components::DescriptorV1& descriptor, Spec& spec, std::string& error) {
    if(descriptor.kind!="guidance" || descriptor.implementationId!="fsim.guidance.velocity_normal_pn.v1" ||
            descriptor.configurationVersion!=1 || descriptor.availability!=components::AvailabilityV1::Available) {
        error="unsupported or unavailable guidance component configuration";return false;
    }
    if(!components::slotsV1(descriptor,{"navigation_gain","cue_deg","activation_ratio","loft_deg","loft_acceleration_g",
            "max_g","life_s","inertial_midcourse","launcher_updates"},{},error)) return false;
    auto candidate=spec;
    constexpr double radians=3.14159265358979323846/180;
    const struct {const char* role;const char* unit;double Spec::* member;double scale=1;} slots[]={
        {"navigation_gain","1",&Spec::navigationGain},{"cue_deg","deg",&Spec::cueRad,radians},
        {"activation_ratio","1",&Spec::activationRatio},{"loft_deg","deg",&Spec::loftRad,radians},
        {"loft_acceleration_g","1",&Spec::loftAccelerationG},{"max_g","1",&Spec::maxG},{"life_s","s",&Spec::lifeS}};
    for(const auto& slot:slots) {
        double value=0;
        if(!components::parameterV1(descriptor,slot.role,slot.unit,value,error)) return false;
        candidate.*slot.member=value*slot.scale;
    }
    double inertial=0,updates=0;
    if(!components::parameterV1(descriptor,"inertial_midcourse","1",inertial,error) ||
            !components::parameterV1(descriptor,"launcher_updates","1",updates,error)) return false;
    if((inertial!=0 && inertial!=1) || (updates!=0 && updates!=1)) {
        error="guidance capability roles need exact zero or one values";return false;
    }
    candidate.capabilityVersion=1;candidate.inertialMidcourse=inertial==1;candidate.launcherUpdates=updates==1;
    candidate.executionVersion=1;candidate.guidanceAlgorithm=1;
    if(!valid(candidate)) {error="guidance component configuration is outside its supported model domain";return false;}
    spec=std::move(candidate);error.clear();return true;
}

bool configureGuidance(const StoreCard& card, Spec& spec, std::string& error) {
    // These are private versioned settings in the already-owned parameter map.
    // Do not change StoreCard/public layouts or retain a mutable/global registry.
    const char* executionKeys[] = {"weapon_execution_version", "guidance_algorithm", "guidance_navigation_gain",
        "guidance_cue_deg", "guidance_activation_ratio", "guidance_loft_deg", "guidance_loft_acceleration_g",
        "control_algorithm", "control_allocator", "control_max_incidence_deg", "control_rate_time_min_s",
        "control_rate_lag_multiplier", "control_incidence_time_min_s", "control_incidence_rate_multiplier",
        "control_actuator_lag_s", "control_actuator_rate_rad_s"};
    // fuze_radius_m predates this versioned block. Alone it must retain its
    // existing caller-owned legacy meaning, not become a malformed new block.
    const char* fuzeKeys[] = {"fuze_model_version", "fuze_proximity_algorithm", "fuze_arming_s",
        "fuze_delay_s", "fuze_station_x_m", "fuze_station_y_m", "fuze_station_z_m"};
    try {
        Spec candidate=spec;
        const auto number=[&](const char* name,const char* unit) {
            const auto p=card.parameters.find(name);
            if (p==card.parameters.end()) throw std::runtime_error(std::string("missing weapon configuration parameter ")+name);
            if (!std::isfinite(p->second.value) || p->second.unit!=unit)
                throw std::runtime_error(std::string("weapon configuration parameter ")+name+" needs a finite value in "+unit);
            return p->second.value;
        };
        const auto supported=[&](const char* name) {
            if (number(name,"1")!=1) throw std::runtime_error(std::string("unsupported weapon configuration selection ")+name);
        };
        const auto any=[&](const auto& names) {
            return std::any_of(std::begin(names),std::end(names),[&](const auto* name){return card.parameters.count(name)!=0;});
        };
        constexpr double radians=3.14159265358979323846/180;
        if (any(executionKeys)) {
            supported("weapon_execution_version"); supported("guidance_algorithm");
            supported("control_algorithm"); supported("control_allocator");
            candidate.executionVersion=1; candidate.guidanceAlgorithm=1;
            candidate.navigationGain=number("guidance_navigation_gain","1");
            candidate.cueRad=number("guidance_cue_deg","deg")*radians;
            candidate.activationRatio=number("guidance_activation_ratio","1");
            candidate.loftRad=number("guidance_loft_deg","deg")*radians;
            candidate.loftAccelerationG=number("guidance_loft_acceleration_g","1");
            auto& c=candidate.control;
            c.version=1; c.algorithm=1; c.allocator=1;
            c.maxIncidenceRad=number("control_max_incidence_deg","deg")*radians;
            c.rateTimeMinS=number("control_rate_time_min_s","s");
            c.rateLagMultiplier=number("control_rate_lag_multiplier","1");
            c.incidenceTimeMinS=number("control_incidence_time_min_s","s");
            c.incidenceRateMultiplier=number("control_incidence_rate_multiplier","1");
            c.actuatorLagS=number("control_actuator_lag_s","s");
            c.actuatorRateRadS=number("control_actuator_rate_rad_s","rad/s");
            if (!valid(candidate) || !(c.maxIncidenceRad>0 && c.maxIncidenceRad<3.14159265358979323846/2)
                || !std::isfinite(c.maxIncidenceRad) || !(c.rateTimeMinS>0) || !(c.rateLagMultiplier>0)
                || !(c.incidenceTimeMinS>0) || !(c.incidenceRateMultiplier>0)
                || !(c.actuatorLagS>0) || !std::isfinite(1/c.actuatorLagS) || !(c.actuatorRateRadS>0))
                throw std::runtime_error("weapon execution configuration is outside its supported model domain");
        }
        if (any(fuzeKeys)) {
            supported("fuze_model_version"); supported("fuze_proximity_algorithm");
            const double length=number("length_m","m"),diameter=number("diameter_m","m");
            candidate.fuzeModelVersion=1; candidate.fuzeAlgorithm=1;
            candidate.armingS=number("fuze_arming_s","s");
            candidate.fuzeM=number("fuze_radius_m","m");
            candidate.fuzeDelayS=number("fuze_delay_s","s");
            candidate.fuzeStationStructuralM=V3{number("fuze_station_x_m","m"),number("fuze_station_y_m","m"),
                                                number("fuze_station_z_m","m")};
            const auto at=*candidate.fuzeStationStructuralM;
            if (!(length>0) || !(diameter>0) || candidate.armingS<0 || candidate.fuzeM<0 || candidate.fuzeDelayS<0
                || !std::isfinite(candidate.lifeS) || !(candidate.armingS<candidate.lifeS)
                || !std::isfinite(candidate.armingS+candidate.fuzeDelayS)
                || !(candidate.armingS+candidate.fuzeDelayS<candidate.lifeS)
                || at.x<0 || at.x>length || std::hypot(at.y,at.z)>diameter/2)
                throw std::runtime_error("explicit fuze configuration is outside its declared model/body/lifetime bounds");
        }
        spec=std::move(candidate); error.clear(); return true;
    } catch (const std::exception& ex) { error=ex.what(); return false; }
}
namespace {
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
    const double activationRange = acquisition * (spec_.executionVersion==1 ? spec_.activationRatio : 1.15);
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
            const double resistance=spec_.seekerModel.version==1?spec_.seekerModel.decoyResistance:
                spec_.seeker.find("imaging")!=std::string::npos?5.0:1.6;
            const double association=spec_.seekerModel.version==1?spec_.seekerModel.associationCosine:.999390827019096;
            const Observation* strongest=observed;
            const V3 heldRay=unit(subtract(observed->position,in.position));
            for (const auto& alternative : in.visible) if (alternative.decoy && eligible(alternative) &&
                    alternative.signal / resistance > observed->signal &&
                    dot(heldRay,unit(subtract(alternative.position,in.position)))>association &&
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
                        && dot(unit(cue), unit(ray)) < (spec_.executionVersion==1 ? std::cos(spec_.cueRad) : cueCosine)) continue;
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
        // Version 1 selects the existing velocity-normal/closing-speed PN law.
        // It does not claim textbook LOS-normal true PN or equipment tuning.
        double gain=0;
        switch (spec_.executionVersion==1 ? spec_.guidanceAlgorithm : 1) {
        case 1: gain=(spec_.executionVersion==1 ? spec_.navigationGain : 3.5)*closing; break;
        default: return Output{};
        }
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
        const double loft = std::sin(std::clamp(spec_.loftRad, 0.0, 0.7)) *
            (spec_.executionVersion==1 ? spec_.loftAccelerationG : 2) * gravity;
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
