#include "simulation/mission_systems/weapon/Autopilot.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace fsim::ms::weapon;
namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
AutopilotInput flight() {
    AutopilotInput in;
    in.massKg = 100;
    in.speedMs = 500;
    in.limitsRad.fill(.3490658503988659);
    in.actuatorLagS = .04;
    in.inertiaKgM2 = {1, 100, 100};
    // Independent synthetic stable tail-controlled plant; SI derivatives.
    in.pitch = {-10000, -2000, -5000, -10000, -2000};
    in.yaw = {-10000, 2000, 5000, -10000, -2000};
    in.rollMomentPerRad = 1000;
    in.rollDamping = -100;
    return in;
}
void level_trim_uses_specific_force_and_gravity() {
    auto in = flight();
    in.demandBody = {0, 0, -9.80665};
    in.gravityBody = {0, 0, 9.80665};
    // Exact steady trim: Z=-10000*a-2000*d, M=-5000*a-10000*d.
    in.alphaRad = 100 * 9.80665 / 9000;
    const auto out = finCommands(in);
    const double trim = -.5 * in.alphaRad;
    require(out.available && std::abs(out.elevator * in.limitsRad[1] - trim) < 1e-12,
            "Level gravity compensation must trim lift without demanding a spurious pitch rate");
    require(std::abs(out.rudder) < 1e-12 && std::abs(out.aileron) < 1e-12,
            "Level trim must not invent yaw or roll demand");
}
void signed_tail_and_canard_authority() {
    for (const double zDelta : {-2000., 2000.}) {
        auto in = flight(); in.pitch.forceControl = zDelta;
        in.demandBody = {0, 10, -10};
        auto out = finCommands(in);
        require(out.available && out.elevator < 0 && out.rudder < 0,
                "Signed moment effectiveness must command nose-up/right for either control-force sign");
        in.pitch.momentControl *= -1;
        out = finCommands(in);
        require(out.available && out.elevator > 0,
                "Reversing physical pitch effectiveness must reverse requested surface direction");
    }
}
void demands_saturate_without_unlimited_authority() {
    auto in = flight();
    in.demandBody = {0, 10000, -10000};
    in.ratesRadS = {100, 100, -100};
    const auto out = finCommands(in);
    require(out.available && std::isfinite(out.elevator) && std::isfinite(out.rudder),
            "Large finite demands must remain finite");
    require(std::abs(out.elevator) <= 1 && std::abs(out.rudder) <= 1 && std::abs(out.aileron) <= 1,
            "The controller may only request bounded normalized surface commands");
}
void no_authority_or_invalid_plant_is_neutral() {
    auto in = flight(); in.demandBody = {0, 20, -20};
    in.pitch.momentControl = 0;
    auto out = finCommands(in);
    require(!out.available && out.elevator == 0 && out.rudder == 0 && out.aileron == 0,
            "Absent control authority must not select any replacement-force mechanism");
    in = flight();
    // A singular force/moment trim system has no independent lift authority.
    in.pitch.forceControl = in.pitch.forceAngle * in.pitch.momentControl / in.pitch.momentAngle;
    require(!finCommands(in).available, "A singular trim plant must fail closed");
    in = flight(); in.speedMs = 0;
    require(!finCommands(in).available, "Zero airspeed must not create aerodynamic authority");
    in = flight(); in.alphaRad = 3.141592653589793;
    require(!finCommands(in).available, "Reversed axial airflow must not use forward-flight control derivatives");
    in = flight(); in.pitch.forceAngle = std::numeric_limits<double>::quiet_NaN();
    require(!finCommands(in).available, "Nonfinite plant values must fail closed");
}
void saturated_request_has_no_windup_history() {
    auto in = flight();
    const auto neutral = finCommands(in);
    in.demandBody = {0, 1e6, -1e6};
    for (unsigned i = 0; i < 100; ++i) (void)finCommands(in);
    in.demandBody = {0, 0, 0};
    const auto recovered = finCommands(in);
    require(neutral.elevator == recovered.elevator && neutral.rudder == recovered.rudder &&
                neutral.aileron == recovered.aileron,
            "Saturation must not retain a hidden integrator or episode history");
}
void finite_lag_synthetic_plant_tracks_a_step() {
    // Independent time-domain plant: finite-lag actuator, actual signed lift
    // and moment, and rotating-body incidence kinematics. No imposed acceleration.
    for (const double controlForce : {-2000., 2000.}) for (const double lag : {.02, .04, .08}) {
        auto in = flight(); in.pitch.forceControl = controlForce; in.actuatorLagS = lag;
        constexpr double demand = -20, dt = .0005;
        in.demandBody = {0, 0, demand};
        double alpha = 0, q = 0, delta = 0, acceleration = 0;
        for (unsigned step = 0; step < 20000; ++step) {
            in.alphaRad = alpha; in.ratesRadS[1] = q;
            const auto out = finCommands(in);
            require(out.available, "Synthetic finite-lag plant must retain finite bounded surface control");
            const double movement = (out.elevator * in.limitsRad[1] - delta) / lag;
            delta += std::clamp(movement, -4., 4.) * dt;
            acceleration = (in.pitch.forceAngle * alpha + in.pitch.forceControl * delta) / in.massKg;
            const double angularAcceleration = (in.pitch.momentAngle * alpha + in.pitch.momentControl * delta +
                                                in.pitch.momentRate * q) / in.inertiaKgM2[1];
            alpha += (q + acceleration / in.speedMs) * dt;
            q += angularAcceleration * dt;
            require(std::isfinite(alpha) && std::abs(alpha) < .3 && std::abs(delta) <= in.limitsRad[1],
                    "Finite-lag closed loop must remain bounded for tail/canard force signs and lag variation");
        }
        require(std::abs(acceleration - demand) < .1,
                "Actual synthetic aerodynamic acceleration must settle within 0.1 m/s2 of the step demand");
    }
}
void two_axis_canard_does_not_require_roll_authority() {
    auto in = flight();
    in.rollMomentPerRad = 0; in.limitsRad[0] = 0; in.ratesRadS[0] = 1;
    in.demandBody = {0, 10, -10};
    const auto out = finCommands(in);
    require(out.available && out.elevator < 0 && out.rudder < 0 && out.aileron == 0,
            "Missing roll effectors must leave valid pitch/yaw controls available with neutral roll command");
}
}
int main() {
    unsigned passed = 0;
    for (const auto test : {level_trim_uses_specific_force_and_gravity, signed_tail_and_canard_authority,
             demands_saturate_without_unlimited_authority, no_authority_or_invalid_plant_is_neutral,
             saturated_request_has_no_windup_history, finite_lag_synthetic_plant_tracks_a_step,
             two_axis_canard_does_not_require_roll_authority}) {
        try { test(); ++passed; }
        catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    }
    std::cout << passed << " missile autopilot cases passed\n";
}
