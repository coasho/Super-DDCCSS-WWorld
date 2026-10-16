#include "simulation/mission_systems/weapon/Guidance.h"

#include <cmath>
#include <iostream>
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
