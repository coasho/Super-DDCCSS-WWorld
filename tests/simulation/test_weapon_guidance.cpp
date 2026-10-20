#include "simulation/mission_systems/weapon/Guidance.h"
#include "simulation/mission_systems/weapon/Seeker.h"
#include "simulation/mission_systems/weapon/TrackKinematics.h"
#include "simulation/mission_systems/Model.h"
#include "foundation/core/Attitude.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace fsim::ms::weapon;
using fsim::ms::WeaponPhase;

namespace {
void require(bool condition, const char* reason) {
    if (!condition) throw std::runtime_error(reason);
}
double length(V3 value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}
Input flight() {
    Input in{};
    in.velocity = {1000, 0, 0};
    in.up = {0, 0, 1};
    in.dtS = 0.05;
    in.supported = true;
    return in;
}
void active_activation_and_reacquisition() {
    Guidance guidance(Spec{"active_radar"});
    auto in = flight();
    in.estimate = Observation{42, {30000, 0, 0}, {-200, 0, 0}};
    auto out = guidance.update(in);
    require(out.phase == WeaponPhase::Midcourse && !out.active && !out.tracking,
            "An active seeker must start in estimate-driven midcourse");
    require(out.timeToGoS > 20 && out.timeToActiveS > 5,
            "Range telemetry must use closing velocity");
    in.ageS = 1;
    in.estimate->position.x = 10000;
    out = guidance.update(in);
    require(out.active && out.phase == WeaponPhase::Seeking && !out.tracking,
            "Entering acquisition range must activate searching without inventing a contact");
    in.visible = {*in.estimate};
    out = guidance.update(in);
    require(out.phase == WeaponPhase::Homing && out.tracking && out.holding == 42,
            "An observed contact must be acquired");
    in.ageS += in.dtS;
    in.visible.clear();
    out = guidance.update(in);
    require(out.phase == WeaponPhase::Lost && !out.tracking && out.holding == 42,
            "Losing an observation must stop tracking while retaining last holding telemetry");
    in.ageS += in.dtS;
    in.visible = {*in.estimate};
    out = guidance.update(in);
    require(out.phase == WeaponPhase::Homing && out.tracking && out.holding == 42,
            "A returning observation must allow reacquisition");
}
void unsupported_estimates_do_not_update_truth() {
    Guidance first(Spec{"active_radar"});
    auto in = flight();
    in.estimate = Observation{42, {30000, 100, 0}, {-200, 5, 0}};
    first.update(in);
    Guidance second = first;
    in.supported = false;
    in.ageS = 2;
    in.estimate = Observation{42, {10, 10000, 0}, {0, 999, 0}};
    auto a = first.update(in);
    in.estimate.reset();
    auto b = second.update(in);
    require(a.phase == WeaponPhase::Midcourse && !a.supported,
            "Loss of midcourse support must retain the inertial estimate");
    require(a.acceleration.x == b.acceleration.x && a.acceleration.y == b.acceleration.y
            && a.acceleration.z == b.acceleration.z && a.timeToGoS == b.timeToGoS,
            "Unsupported input estimates must never refresh hidden truth");
    require(a.timeToGoS < 25 && a.timeToGoS > 22,
            "An inertial estimate must extrapolate the last accepted target velocity");
}
void sarh_requires_illumination() {
    Guidance guidance(Spec{"semi_active_radar"});
    auto in = flight();
    in.illuminated = true;
    in.estimate = Observation{7, {10000, 0, 0}, {0, 0, 0}};
    in.visible = {*in.estimate};
    auto out = guidance.update(in);
    require(out.tracking && out.supported && !out.active,
            "SARH must home using illumination without active transmission");
    in.illuminated = false;
    out = guidance.update(in);
    require(!out.tracking && !out.supported && out.phase == WeaponPhase::Lost,
            "Visible truth cannot sustain SARH after illumination is removed");
    in.illuminated = true;
    out = guidance.update(in);
    require(out.tracking && out.phase == WeaponPhase::Homing,
            "SARH must reacquire when illumination and observations return");
}
void infrared_requires_observations_and_prediction_gate() {
    Guidance guidance(Spec{"infrared"});
    auto in = flight();
    in.estimate = Observation{8, {8000, 0, 0}, {0, 0, 0}};
    auto out = guidance.update(in);
    require(out.phase == WeaponPhase::Seeking && !out.tracking && !out.active,
            "An infrared seeker must not home on launcher estimates alone");
    in.visible = {{9, {8000, 4000, 0}, {0, 0, 0}, true}};
    out = guidance.update(in);
    require(!out.tracking, "A decoy far outside the prediction gate must not be acquired");
    in.visible.push_back({10, {8000, 20, 0}, {0, 0, 0}, true});
    out = guidance.update(in);
    require(out.tracking && out.holding == 10,
            "A suitable observed decoy near the estimate can be acquired");
}
void acceleration_is_bounded_and_finite() {
    Spec spec{"infrared"};
    spec.maxG = 3;
    Guidance guidance(spec);
    auto in = flight();
    in.visible = {{12, {1000, 1000, 0}, {0, 500, 0}}};
    const auto out = guidance.update(in);
    require(out.tracking && out.acceleration.y > 0,
            "Proportional navigation must turn towards the crossing contact");
    require(length(out.acceleration) <= 3 * 9.80665 + 1e-9,
            "Guidance including gravity compensation must respect the acceleration limit");
    in.velocity = {};
    in.visible.front().position = {};
    const auto zero = guidance.update(in);
    require(std::isfinite(length(zero.acceleration)),
            "Zero speed and coincident observations must not produce NaN");
}
void swept_proximity_and_arming() {
    double fraction = -1;
    const double miss = sweptMissDistance({-1000, 0, 0}, {1000, 0, 0},
                                          {0, 8, 0}, {0, 8, 0}, &fraction);
    require(std::abs(miss - 8) < 1e-12 && std::abs(fraction - 0.5) < 1e-12,
            "A high-speed crossing must detect the interior closest approach");
    require(std::abs(sweptMissDistance({0, 0, 0}, {1, 0, 0},
                                     {10, 0, 0}, {11, 0, 0}) - 10) < 1e-12,
            "Equal velocities must return finite separation");
    require(std::abs(sweptMissDistance({0, 0, 0}, {-10, 0, 0},
                                     {5, 0, 0}, {5, 0, 0}, &fraction) - 5) < 1e-12
            && fraction == 0, "Closest approach must clamp to segment boundaries");
    Spec spec{"active_radar"};
    require(!armed(0.99, spec) && armed(1, spec) && !armed(-1, spec),
            "The fuze must remain safe before arming time");
}
void no_guidance_without_a_seeker() {
    Guidance guidance(Spec{});
    auto in = flight();
    in.estimate = Observation{2, {100, 100, 0}, {0, 0, 0}};
    in.visible = {*in.estimate};
    const auto out = guidance.update(in);
    require(out.phase == WeaponPhase::Unguided && !out.tracking && length(out.acceleration) == 0,
            "An inert store must never generate steering commands");
}
void imaging_infrared_and_signal_discrimination() {
    auto in=flight();
    in.visible={{12,{1000,0,0},{0,0,0}}};
    Guidance imaging(Spec{"imaging_infrared"});
    require(imaging.update(in).tracking,"Imaging infrared must use its native seeker, not remain unguided");
    Guidance heat(Spec{"infrared"});
    require(heat.update(in).holding==12,"Infrared must initially hold the aircraft observation");
    in.ageS+=in.dtS;
    in.visible.push_back({50,{1000,1,0},{0,0,0},true,4.0});
    require(heat.update(in).holding==50,"A stronger flare in the held cue must compete without a random roll");
    require(imaging.update(in).holding==12,"Imaging discrimination must reject the same moderate flare");
    in.visible.back().signal=6.0;
    require(imaging.update(in).holding==50,"A fresh stronger flare must be able to capture an imaging seeker");
    Guidance outside(Spec{"infrared"});
    in.visible.resize(1); outside.update(in);
    in.visible.push_back({50,{1000,500,0},{0,0,0},true,100.0});
    require(outside.update(in).holding==12,"A strong flare outside the association gate must not steal a hold");
}
void on_rail_cue_requires_current_observation() {
    auto in=flight(); in.supported=false; in.launchCue=12;
    in.visible={{12,{5000,0,0},{250,0,0}}, {13,{940,342,0},{250,0,0}}};
    Guidance heat(Spec{"imaging_infrared"});
    require(heat.update(in).holding==12,"Initial acquisition must preserve the observed on-rail contact");
    in.visible.erase(in.visible.begin());
    require(!heat.update(in).tracking,"A launch cue must not authorize an invisible target hold");
    in.visible.clear();
    require(!heat.update(in).tracking,"A launch cue must not supply current observations after target loss");
}
void invalid_timing_does_not_mutate_guidance() {
    const double invalidDurations[] = {-0.01, std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::infinity()};
    for (double invalid : invalidDurations) {
        Guidance actual(Spec{"active_radar"});
        auto in = flight();
        in.ageS = 2;
        in.estimate = Observation{42, {30000, 0, 0}, {-200, 0, 0}};
        actual.update(in);
        Guidance reference = actual;
        in.ageS = 100;
        in.dtS = invalid;
        in.estimate = Observation{99, {1000, 0, 0}, {0, 0, 0}};
        in.visible = {*in.estimate};
        const auto refused = actual.update(in);
        require(refused.phase == WeaponPhase::Unguided && !refused.active && !refused.tracking
                    && refused.holding == 0 && length(refused.acceleration) == 0,
                "Invalid step duration must fail closed before activation or acquisition");
        in = flight();
        in.ageS = 3;
        in.supported = false;
        const auto recovered = actual.update(in);
        const auto expected = reference.update(in);
        require(recovered.phase == expected.phase && recovered.holding == expected.holding
                    && recovered.acceleration.x == expected.acceleration.x
                    && recovered.acceleration.y == expected.acceleration.y
                    && recovered.acceleration.z == expected.acceleration.z
                    && recovered.timeToGoS == expected.timeToGoS,
                "An invalid future update must not change the accepted estimate or launch age");
    }
}
void invalid_launch_age_does_not_mutate_guidance() {
    const double invalidAges[] = {-0.01, 1.0, std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()};
    for (double invalid : invalidAges) {
        Guidance actual(Spec{"active_radar"});
        auto in = flight();
        in.ageS = 2;
        in.estimate = Observation{42, {30000, 0, 0}, {-200, 0, 0}};
        actual.update(in);
        Guidance reference = actual;
        in.ageS = invalid;
        in.estimate = Observation{99, {1000, 0, 0}, {0, 0, 0}};
        in.visible = {*in.estimate};
        const auto refused = actual.update(in);
        require(refused.phase == WeaponPhase::Unguided && !refused.active && !refused.tracking
                    && refused.holding == 0 && length(refused.acceleration) == 0,
                "Negative, nonfinite or reversed launch age must fail closed before state mutation");
        in = flight();
        in.ageS = 3;
        in.supported = false;
        const auto recovered = actual.update(in);
        const auto expected = reference.update(in);
        require(recovered.phase == expected.phase && recovered.holding == expected.holding
                    && recovered.acceleration.x == expected.acceleration.x
                    && recovered.acceleration.y == expected.acceleration.y
                    && recovered.acceleration.z == expected.acceleration.z
                    && recovered.timeToGoS == expected.timeToGoS,
                "Rejected launch age must leave the prior estimate and activation state unchanged");
    }
    Guidance sameAge(Spec{"infrared"});
    auto in = flight();
    in.visible = {{12, {1000, 0, 0}, {0, 0, 0}}};
    require(sameAge.update(in).tracking && sameAge.update(in).tracking,
            "Repeated observations at the same launch age remain supported");
}
void invalid_guidance_specification_is_safe() {
    const double invalidValues[] = {-1, std::numeric_limits<double>::quiet_NaN(),
                                   std::numeric_limits<double>::infinity()};
    for (unsigned field = 0; field != 7; ++field) for (double invalid : invalidValues) {
        Spec spec{"infrared"};
        double* fields[] = {&spec.acquireM, &spec.gimbalRad, &spec.maxG, &spec.lifeS,
                            &spec.armingS, &spec.fuzeM, &spec.loftRad};
        *fields[field] = invalid;
        // Negative loft retains the existing clamped loft convention; all other
        // fields represent nonnegative physical magnitudes or durations.
        if (field == 6 && invalid < 0) continue;
        Guidance guidance(spec);
        auto in = flight();
        in.visible = {{12, {1000, 0, 0}, {0, 0, 0}}};
        const auto out = guidance.update(in);
        require(out.phase == WeaponPhase::Unguided && !out.tracking
                    && length(out.acceleration) == 0 && !armed(10, spec),
                "Invalid model parameters must neither steer nor arm a weapon");
    }
    Spec wide{"infrared"};
    wide.gimbalRad = 3.141592653589793 + 0.01;
    Guidance guidance(wide);
    auto in = flight();
    in.visible = {{12, {1000, 0, 0}, {0, 0, 0}}};
    require(guidance.update(in).phase == WeaponPhase::Unguided,
            "An impossible gimbal angle must not silently become an unrestricted seeker");
}
void unavailable_timing_estimates_are_unknown() {
    auto in = flight();
    in.supported = false;
    Guidance radar(Spec{"active_radar"});
    const auto searching = radar.update(in);
    require(std::isnan(searching.timeToGoS) && searching.timeToActiveS == 0,
            "An active search has no time to go, while its activation has already occurred");

    Guidance midcourse(Spec{"active_radar"});
    in.supported = true;
    in.estimate = Observation{42, {30000, 0, 0}, {1200, 0, 0}};
    const auto opening = midcourse.update(in);
    require(std::isnan(opening.timeToGoS) && std::isnan(opening.timeToActiveS),
            "A nonclosing midcourse estimate must not fabricate zero timing estimates");

    for (const auto* seeker : {"infrared", "semi_active_radar"}) {
        Guidance passive(Spec{seeker});
        in = flight();
        in.illuminated = true;
        in.visible = {{12, {1000, 0, 0}, {0, 0, 0}}};
        const auto homing = passive.update(in);
        require(homing.tracking && homing.timeToGoS > 0 && std::isnan(homing.timeToActiveS),
                "A seeker without an active-radar transition has no time-to-active estimate");
    }
    Guidance inert(Spec{});
    const auto unguided = inert.update(flight());
    require(std::isnan(unguided.timeToGoS) && std::isnan(unguided.timeToActiveS),
            "An unguided or refused update must publish unknown timing estimates");
}
void infrared_reach_preserves_admitted_assumptions() {
    require(infraredAcquisitionRangeM(13000, 1, false) == 13000
                && std::abs(infraredAcquisitionRangeM(13000, -1, false) - 5850) < 1e-9
                && std::abs(infraredAcquisitionRangeM(13000, 0, false) - 9425) < 1e-9
                && std::abs(infraredAcquisitionRangeM(13000, 1, true) - 17550) < 1e-9,
            "Shared infrared reach must preserve the existing aspect and afterburner assumptions");
    require(infraredAcquisitionRangeM(13000, 2, false) == 13000
                && std::abs(infraredAcquisitionRangeM(13000, -2, false) - 5850) < 1e-9,
            "Aspect cosine must be clamped to its geometric domain");
    for (double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()})
        require(infraredAcquisitionRangeM(invalid, 1, true) == 0,
                "Invalid infrared baseline range must not create a sensor envelope");
    require(infraredAcquisitionRangeM(13000, std::numeric_limits<double>::quiet_NaN(), true) == 0
                && infraredAcquisitionRangeM(13000, std::numeric_limits<double>::infinity(), true) == 0
                && infraredAcquisitionRangeM(std::numeric_limits<double>::max(), 1, true) == 0,
            "Nonfinite aspect or overflowing reach must fail closed");
}
void admitted_observation_range_is_defensively_checked() {
    Spec spec{"infrared"};
    spec.acquireM = 13000;
    auto in = flight();
    in.supported = false;
    in.launchCue = 12;
    in.visible = {{12, {15000, 0, 0}, {250, 0, 0}, false, 1,
                   infraredAcquisitionRangeM(spec.acquireM, 1, true)}};
    Guidance heat(spec);
    require(heat.update(in).tracking,
            "A current observed on-rail contact inside admitted afterburner reach must survive release");
    in.ageS += in.dtS;
    in.visible.front().position.x = 17551;
    require(!heat.update(in).tracking,
            "Per-observation sensor admission must still enforce its geometric range cap");

    for (double invalid : {-1.0, std::numeric_limits<double>::infinity()}) {
        Guidance invalidRange(spec);
        in.visible.front().position.x = 1000;
        in.visible.front().admittedRangeM = invalid;
        require(!invalidRange.update(in).tracking,
                "Explicit negative or infinite observation reach must not fall back to the model range");
    }
    Guidance legacy(spec);
    in.visible.front().position.x = 15000;
    in.visible.front().admittedRangeM = std::numeric_limits<double>::quiet_NaN();
    require(!legacy.update(in).tracking,
            "An observation without a specific admission must retain the baseline defensive range cap");
    Guidance narrow(spec);
    in.visible.front().position.x = 5000;
    in.visible.front().admittedRangeM = 4000;
    require(!narrow.update(in).tracking,
            "An admitted range below the baseline must narrow observation eligibility");
}
void seeker_gimbal_uses_explicit_body_boresight() {
    Spec spec{"infrared"};
    spec.gimbalRad = 0.5235987755982988;
    auto in = flight();
    in.supported = false;
    in.boresight = V3{0, 1, 0};
    in.visible = {{12, {0, 1000, 0}, {0, 0, 0}}};
    Guidance bodyAligned(spec);
    require(bodyAligned.update(in).tracking,
            "A body-aligned seeker observation must remain eligible when velocity points elsewhere");
    Guidance velocityAligned(spec);
    in.visible.front().position = {1000, 0, 0};
    require(!velocityAligned.update(in).tracking,
            "A velocity-aligned observation outside the body gimbal must be rejected");
    Guidance stationary(spec);
    in.velocity = {};
    in.visible.front().position = {0, 1000, 0};
    require(stationary.update(in).tracking,
            "A finite explicit body boresight must define the gimbal even without translational velocity");
    Guidance stationaryOutside(spec);
    in.visible.front().position = {1000, 0, 0};
    require(!stationaryOutside.update(in).tracking,
            "Zero velocity must not disable an explicitly supplied body gimbal");
}
void invalid_boresight_does_not_mutate_guidance() {
    const V3 invalidDirections[] = {{}, {std::numeric_limits<double>::quiet_NaN(), 0, 0},
                                     {0, std::numeric_limits<double>::infinity(), 0}};
    for (const V3 invalid : invalidDirections) {
        Guidance actual(Spec{"active_radar"});
        auto in = flight();
        in.ageS = 2;
        in.estimate = Observation{42, {30000, 0, 0}, {-200, 0, 0}};
        actual.update(in);
        Guidance reference = actual;
        in.ageS = 100;
        in.boresight = invalid;
        in.estimate = Observation{99, {1000, 0, 0}, {0, 0, 0}};
        in.visible = {*in.estimate};
        const auto refused = actual.update(in);
        require(refused.phase == WeaponPhase::Unguided && !refused.active && !refused.tracking,
                "An explicitly invalid body boresight must fail closed before state mutation");
        in = flight();
        in.ageS = 3;
        in.supported = false;
        const auto recovered = actual.update(in);
        const auto expected = reference.update(in);
        require(recovered.phase == expected.phase && recovered.timeToGoS == expected.timeToGoS
                    && recovered.acceleration.x == expected.acceleration.x
                    && recovered.acceleration.y == expected.acceleration.y
                    && recovered.acceleration.z == expected.acceleration.z,
                "A rejected boresight must not poison the accepted estimate or timing state");
    }
}

fsim::sim::VehicleState measuredTrack(double latitude, double longitude, V3 ned) {
    fsim::sim::VehicleState state;
    state.simTime = 17;
    state.latitudeRad = latitude;
    state.longitudeRad = longitude;
    state.positionEcef[0] = 30000;
    state.positionEcef[1] = 100;
    state.velocityNedMs[0] = ned.x;
    state.velocityNedMs[1] = ned.y;
    state.velocityNedMs[2] = ned.z;
    state.eulerRad[0] = .7; // A synthetic track must not retain a measured-looking roll.
    state.eulerRad[1] = std::atan2(-ned.z, std::hypot(ned.x, ned.y));
    state.eulerRad[2] = std::atan2(ned.y, ned.x);
    return state;
}

void checkTrackFrame(fsim::sim::VehicleState state, V3 expectedEcef) {
    const auto before = state;
    require(completeTrackKinematics(state), "Finite measured track kinematics must be completed");
    const double speed = std::hypot(before.velocityNedMs[0], before.velocityNedMs[1], before.velocityNedMs[2]);
    const double tolerance = 1e-12 * std::max(1., speed);
    double actual[3];
    fsim::ms::geom::velocityEcef(state, actual);
    require(std::abs(actual[0] - expectedEcef.x) <= tolerance
                && std::abs(actual[1] - expectedEcef.y) <= tolerance
                && std::abs(actual[2] - expectedEcef.z) <= tolerance,
            "Existing body-state consumers must recover the measured NED velocity in ECEF");
    require(std::abs(state.velocityBodyMs[0] - speed) <= tolerance
                && std::abs(state.velocityBodyMs[1]) <= tolerance
                && std::abs(state.velocityBodyMs[2]) <= tolerance && state.eulerRad[0] == 0,
            "The synthetic frame must be the derived zero-roll velocity frame");
    for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) {
        double dot = 0;
        for (int row = 0; row < 3; ++row)
            dot += state.rotationBodyToEcef[3 * row + a] * state.rotationBodyToEcef[3 * row + b];
        require(std::abs(dot - (a == b ? 1. : 0.)) < 1e-12,
                "The derived body-to-ECEF frame must remain orthonormal");
    }
    double fromQuaternion[9];
    fsim::core::attitude::bodyToEcefFromQuaternion(state.attitudeEcefToBody, fromQuaternion);
    for (int k = 0; k < 9; ++k)
        require(std::abs(fromQuaternion[k] - state.rotationBodyToEcef[k]) < 1e-12,
                "The ECEF-to-body quaternion and body-to-ECEF rotation must describe the same frame");
    require(state.simTime == before.simTime && state.positionEcef[0] == before.positionEcef[0]
                && state.positionEcef[1] == before.positionEcef[1] && state.positionEcef[2] == before.positionEcef[2]
                && state.velocityNedMs[0] == before.velocityNedMs[0]
                && state.velocityNedMs[1] == before.velocityNedMs[1]
                && state.velocityNedMs[2] == before.velocityNedMs[2],
            "Completing a derived frame must preserve the measured sample and its timestamp");
}

void measured_track_velocity_and_rotated_frames_are_consistent() {
    constexpr double halfPi = 1.5707963267948966;
    checkTrackFrame(measuredTrack(0, 0, {10, 20, 30}), {-30, 20, 10});
    checkTrackFrame(measuredTrack(0, halfPi, {10, 20, 30}), {-20, -30, 10});
    const double latitude = .6, longitude = -1.3;
    const V3 ned{123, -217, 46};
    const V3 expected{-std::sin(latitude) * std::cos(longitude) * ned.x - std::sin(longitude) * ned.y
                         - std::cos(latitude) * std::cos(longitude) * ned.z,
                     -std::sin(latitude) * std::sin(longitude) * ned.x + std::cos(longitude) * ned.y
                         - std::cos(latitude) * std::sin(longitude) * ned.z,
                     std::cos(latitude) * ned.x - std::sin(latitude) * ned.z};
    checkTrackFrame(measuredTrack(latitude, longitude, ned), expected);
    checkTrackFrame(measuredTrack(0, 0, {1e200, -1e200, 1e200}), {-1e200, -1e200, 1e200});
}

void zero_and_polar_track_frames_remain_finite() {
    constexpr double halfPi = 1.5707963267948966;
    checkTrackFrame(measuredTrack(.6, -1.3, {}), {});
    checkTrackFrame(measuredTrack(halfPi, 0, {10, 20, 30}), {-10, 20, -30});
    checkTrackFrame(measuredTrack(-halfPi, 0, {10, 20, 30}), {10, 20, 30});
    checkTrackFrame(measuredTrack(halfPi, halfPi, {}), {});
}

void invalid_track_kinematics_are_atomic() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (int field = 0; field < 8; ++field) {
        auto state = measuredTrack(.6, -1.3, {123, -217, 46});
        if (field == 0) state.latitudeRad = nan;
        if (field == 1) state.longitudeRad = inf;
        if (field == 2) state.velocityNedMs[0] = inf;
        if (field == 3) state.velocityNedMs[2] = nan;
        if (field == 4) state.eulerRad[1] = nan;
        if (field == 5) state.eulerRad[2] = inf;
        if (field == 6) state.latitudeRad = 3.141592653589793;
        if (field == 7) state.velocityNedMs[0] = state.velocityNedMs[1] = std::numeric_limits<double>::max();
        unsigned char before[sizeof state];
        std::memcpy(before, &state, sizeof state);
        require(!completeTrackKinematics(state), "Invalid or unrepresentable track kinematics must be refused");
        require(std::memcmp(before, &state, sizeof state) == 0,
                "A refused track conversion must preserve the complete existing state");
    }
}

void completed_track_velocity_drives_unsupported_extrapolation() {
    auto state = measuredTrack(0, 0, {0, 0, 200}); // ECEF velocity {-200, 0, 0}.
    require(completeTrackKinematics(state), "Measured approaching track must have usable kinematics");
    double measuredVelocity[3];
    fsim::ms::geom::velocityEcef(state, measuredVelocity);
    Guidance guidance(Spec{"active_radar"});
    auto in = flight();
    in.ageS = 2;
    in.estimate = Observation{42, {30000, 100, 0},
        {measuredVelocity[0], measuredVelocity[1], measuredVelocity[2]}};
    guidance.update(in);
    in.ageS = 4;
    in.supported = false;
    in.estimate.reset();
    const auto output = guidance.update(in);
    const double expected = (29600. * 29600. + 100. * 100.) / (1200. * 29600.);
    require(!output.supported && output.phase == WeaponPhase::Midcourse
                && std::abs(output.timeToGoS - expected) < 1e-10,
            "Lost support must extrapolate measured target velocity instead of a stationary synthetic body");
}
}

int main(int argc, char** argv) {
    const struct { const char* name; void (*run)(); } tests[] = {
        {"active", active_activation_and_reacquisition},
        {"stale", unsupported_estimates_do_not_update_truth},
        {"sarh", sarh_requires_illumination},
        {"infrared", infrared_requires_observations_and_prediction_gate},
        {"acceleration", acceleration_is_bounded_and_finite},
        {"fuze", swept_proximity_and_arming},
        {"unguided", no_guidance_without_a_seeker},
        {"discrimination", imaging_infrared_and_signal_discrimination},
        {"on-rail cue", on_rail_cue_requires_current_observation},
        {"invalid duration", invalid_timing_does_not_mutate_guidance},
        {"invalid launch age", invalid_launch_age_does_not_mutate_guidance},
        {"invalid spec", invalid_guidance_specification_is_safe},
        {"unknown timing", unavailable_timing_estimates_are_unknown},
        {"infrared reach", infrared_reach_preserves_admitted_assumptions},
        {"observation range", admitted_observation_range_is_defensively_checked},
        {"body boresight", seeker_gimbal_uses_explicit_body_boresight},
        {"invalid boresight", invalid_boresight_does_not_mutate_guidance},
        {"track frames", measured_track_velocity_and_rotated_frames_are_consistent},
        {"track zero polar", zero_and_polar_track_frames_remain_finite},
        {"invalid track", invalid_track_kinematics_are_atomic},
        {"track extrapolation", completed_track_velocity_drives_unsupported_extrapolation},
    };
    for (const auto& test : tests) {
        if (argc > 1 && std::string(argv[1]) != test.name) continue;
        try { test.run(); std::cout << "PASS " << test.name << '\n'; }
        catch (const std::exception& error) {
            std::cerr << "FAIL " << test.name << ": " << error.what() << '\n';
            return 1;
        }
    }
}
