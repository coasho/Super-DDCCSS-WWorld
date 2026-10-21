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
ControlStopsV1 actualActuators() {
    ControlStopsV1 native; native.actuatorVersion=1;
    native.channelRateRadS=4; native.finRateRadS={4,4,4,4};
    return native;
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
void asymmetric_native_stop_normalization_preserves_requested_radians() {
    auto in=flight(); in.limitsRad.fill(.3);
    in.demandBody={0,0,-9.80665}; in.gravityBody={0,0,9.80665};
    in.alphaRad=100*9.80665/9000;
    FinControlConfigV1 config; config.version=1;
    auto stops=actualActuators(); stops.version=1;
    stops.lowerRad.fill(-.1); stops.upperRad.fill(.3);
    const auto out=finCommands(in,config,stops);
    // Independent JSBSim zero-centred scale: negative input uses |lower|.
    const double actual=out.elevator*(out.elevator<0 ? .1 : .3);
    require(out.available && std::abs(actual+.5*in.alphaRad)<1e-12,
            "Asymmetric native stops must realize the requested negative trim radians, not a maximum-absolute-stop surrogate");
}
void configured_control_response_and_refusals_are_executed() {
    auto in=flight(); in.ratesRadS[0]=.3; in.rollDamping=0;
    FinControlConfigV1 config; config.version=1;
    const auto native=actualActuators();
    const auto ordinary=finCommands(in,config,native);
    config.rateTimeMinS=.24;
    const auto slower=finCommands(in,config,native);
    require(ordinary.available && slower.available && std::abs(slower.aileron-.5*ordinary.aileron)<1e-12,
            "The configured rate-response minimum must change the synthetic servo law rather than only metadata");
    for(int defect=0;defect<5;++defect) {
        auto bad=config;
        if(defect==0) bad.version=2;
        if(defect==1) bad.algorithm=2;
        if(defect==2) bad.allocator=2;
        if(defect==3) bad.rateTimeMinS=0;
        if(defect==4) bad.maxIncidenceRad=std::numeric_limits<double>::quiet_NaN();
        const auto out=finCommands(in,bad,native);
        require(!out.available && out.aileron==0 && out.elevator==0 && out.rudder==0,
                "Explicit unsupported or malformed control configuration must refuse with neutral commands");
    }
    auto badStops=native; badStops.version=1;
    require(!finCommands(in,config,badStops).available,"Present malformed signed stops must not fall back to legacy normalization");
    badStops.version=2;
    require(!finCommands(in,config,badStops).available,"Unknown signed-stop metadata must refuse");
}
void configured_native_actuator_mismatch_is_unavailable() {
    auto in=flight(); FinControlConfigV1 config; config.version=1;
    const auto native=actualActuators();
    require(finCommands(in,config,native).available,"Matching native actuator inputs must retain configured control");
    config.actuatorLagS=.08;
    require(!finCommands(in,config,native).available,
            "A configured actuator time constant must not masquerade as the different actual native lag");
    config.actuatorLagS=.04;
    auto wrong=native; wrong.channelRateRadS=3;
    require(!finCommands(in,config,wrong).available,"A native channel-rate mismatch must refuse");
    wrong=native; wrong.finRateRadS[2]=3;
    require(!finCommands(in,config,wrong).available,"Every physical fin's actual rate must match the selected plant");
    wrong=native; wrong.finRateRadS.clear();
    require(!finCommands(in,config,wrong).available,"Configured native fin dynamics cannot be inferred from an empty inventory");
    wrong=native; wrong.actuatorVersion=0;
    require(!finCommands(in,config,wrong).available,"A configured profile requires explicit native actuator metadata");
    config.actuatorLagS=in.actuatorLagS=.08; config.actuatorRateRadS=3.25;
    auto selected=native; selected.channelRateRadS=3.25; selected.finRateRadS.assign(4,3.25);
    require(finCommands(in,config,selected).available,
            "Matching nondefault card/native actuator inputs must execute the selected finite plant");
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
void nonlinear_native_loads_do_not_become_hidden_tracking_bias() {
    // Independent synthetic nonlinear plant. Terms are deliberately synthetic,
    // not fitted to an engagement. No direct acceleration or ideal reaction.
    for (const double controlForce : {-2000.,2000.}) for (const double lag : {.02,.04,.08}) {
        auto in=flight(); in.pitch.forceControl=controlForce; in.actuatorLagS=lag;
        in.demandBody={0,8,-20};
        double alpha=0,beta=0,q=0,r=0,elevator=0,rudder=0,az=0,ay=0;
        constexpr double dt=.0005;
        for (unsigned step=0;step<30000;++step) {
            const double fz=in.pitch.forceAngle*alpha+in.pitch.forceControl*elevator-12000*alpha*std::abs(alpha);
            const double fy=in.yaw.forceAngle*beta+in.yaw.forceControl*rudder-12000*beta*std::abs(beta);
            const double my=in.pitch.momentAngle*alpha+in.pitch.momentControl*elevator+in.pitch.momentRate*q-30000*alpha*std::abs(alpha);
            const double mz=in.yaw.momentAngle*beta+in.yaw.momentControl*rudder+in.yaw.momentRate*r+30000*beta*std::abs(beta);
            in.alphaRad=alpha; in.betaRad=beta; in.ratesRadS={0,q,r};
            in.measured=AeroFeedback{{0,fy,fz},{0,my,mz},{0,elevator,rudder}};
            const auto out=finCommands(in);
            require(out.available,"Synthetic nonlinear plant must retain valid control");
            az=fz/in.massKg; ay=fy/in.massKg;
            alpha+=(q+az/in.speedMs)*dt; beta+=(-r+ay/in.speedMs)*dt;
            q+=my/in.inertiaKgM2[1]*dt; r+=mz/in.inertiaKgM2[2]*dt;
            elevator+=std::clamp((out.elevator*in.limitsRad[1]-elevator)/lag,-4.,4.)*dt;
            rudder+=std::clamp((out.rudder*in.limitsRad[2]-rudder)/lag,-4.,4.)*dt;
            require(std::isfinite(alpha) && std::isfinite(beta) && std::hypot(alpha,beta)<.3,
                    "Native-load feedback must remain bounded with finite actuator lag/rate");
        }
        std::cout << "NONLINEAR: control_force=" << controlForce << " lag_s=" << lag << " az=" << az << " ay=" << ay << '\n';
        require(std::abs(az+20)<.1 && std::abs(ay-8)<.1,
                "Known nonlinear native loads must not leave a steady acceleration error in either control axis");
    }
}
void aligned_load_feedback_preserves_linear_solution_and_measured_positions() {
    auto in=flight(); in.alphaRad=.08; in.betaRad=-.04; in.ratesRadS={.1,.02,-.01}; in.demandBody={0,6,-15};
    const auto plain=finCommands(in);
    const std::array<double,3> actual={.02,-.07,.03};
    AeroFeedback m;
    m.controlRad=actual;
    m.forceBodyN={0,in.yaw.forceAngle*in.betaRad+in.yaw.forceControl*actual[2],in.pitch.forceAngle*in.alphaRad+in.pitch.forceControl*actual[1]};
    m.momentBodyNm={in.rollMomentPerRad*actual[0]+in.rollDamping*in.ratesRadS[0],
        in.pitch.momentAngle*in.alphaRad+in.pitch.momentControl*actual[1]+in.pitch.momentRate*in.ratesRadS[1],
        in.yaw.momentAngle*in.betaRad+in.yaw.momentControl*actual[2]+in.yaw.momentRate*in.ratesRadS[2]};
    in.measured=m;
    const auto aligned=finCommands(in);
    require(aligned.available && std::abs(aligned.elevator-plain.elevator)<1e-12 &&
            std::abs(aligned.rudder-plain.rudder)<1e-12 && std::abs(aligned.aileron-plain.aileron)<1e-12,
            "Zero nonlinear residual must preserve the existing linear solution for all axes");
    // Exact constant-load trim independent of the commanded actuator position.
    in=flight(); in.alphaRad=.1; in.demandBody={0,0,-10}; in.gravityBody={0,0,10};
    constexpr double delta=-.08;
    in.measured=AeroFeedback{{0,0,-1000},{0,0,0},{0,delta,0}};
    const auto trim=finCommands(in);
    require(trim.available && std::abs(trim.elevator*in.limitsRad[1]-delta)<1e-12,
            "A measured constant-load equilibrium must remain trimmed at the actual fin position");
    for (int field=0;field<3;++field) for (double bad : {std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
        auto invalid=in;
        (field==0?invalid.measured->forceBodyN:field==1?invalid.measured->momentBodyNm:invalid.measured->controlRad)[1]=bad;
        require(!finCommands(invalid).available,"Invalid load/position feedback must fail closed");
    }
    in.measured->momentBodyNm[1]=std::numeric_limits<double>::max();
    require(!finCommands(in).available,"Overflowing residual trim must fail closed without usable commands");
}
void coupled_rigid_body_rates_follow_the_requested_servo_response() {
    // Independent Euler rigid-body oracle, deliberately using no direct
    // control force. Zero incidence/demand then requests zero angular rates,
    // so the stated 120 ms rate servo must produce omega_dot = -omega / .12.
    // Actual aerodynamic torque still comes only from the returned surfaces.
    // Synthetic inertias/rates are not equipment data or encounter tuning.
    for (const auto inertia : {std::array<double,3>{2,100,110},
                               std::array<double,3>{110,2,100},
                               std::array<double,3>{100,110,2}}) {
        for (const double pSign : {-1.,1.}) for (const double qSign : {-1.,1.})
            for (const double rSign : {-1.,1.}) for (const bool measured : {false,true}) {
                auto in=flight();
                in.inertiaKgM2=inertia;
                in.pitch.forceControl=in.yaw.forceControl=0;
                in.ratesRadS={pSign*.3,qSign*.06,rSign*.08};
                if (measured) {
                    // Perfect SAME-frame aerodynamic-load knowledge is not
                    // knowledge of the rigid body's inertial coupling torque.
                    const std::array<double,3> actual={.01,-.02,.015};
                    in.measured=AeroFeedback{{0,0,0},{
                        in.rollMomentPerRad*actual[0]+in.rollDamping*in.ratesRadS[0],
                        in.pitch.momentControl*actual[1]+in.pitch.momentRate*in.ratesRadS[1],
                        in.yaw.momentControl*actual[2]+in.yaw.momentRate*in.ratesRadS[2]},actual};
                }
                const auto out=finCommands(in);
                require(out.available && std::abs(out.aileron)<1 &&
                            std::abs(out.elevator)<1 && std::abs(out.rudder)<1,
                        "Coupled-rate fixture must retain unsaturated finite control authority");
                const auto& w=in.ratesRadS;
                const std::array<double,3> torque={
                    in.rollMomentPerRad*out.aileron*in.limitsRad[0]+in.rollDamping*w[0],
                    in.pitch.momentControl*out.elevator*in.limitsRad[1]+in.pitch.momentRate*w[1],
                    in.yaw.momentControl*out.rudder*in.limitsRad[2]+in.yaw.momentRate*w[2]};
                // J * omega_dot + omega cross (J * omega) = actual torque.
                const std::array<double,3> gyroscopic={
                    (inertia[2]-inertia[1])*w[1]*w[2],
                    (inertia[0]-inertia[2])*w[0]*w[2],
                    (inertia[1]-inertia[0])*w[0]*w[1]};
                for (std::size_t axis=0;axis<3;++axis) {
                    const double achieved=(torque[axis]-gyroscopic[axis])/inertia[axis];
                    const double expected=-w[axis]/.12;
                    if (std::abs(achieved-expected)>1e-11)
                        std::cerr << "COUPLED: measured=" << measured << " axis=" << axis
                                  << " inertia=" << inertia[0] << ',' << inertia[1] << ',' << inertia[2]
                                  << " rates=" << w[0] << ',' << w[1] << ',' << w[2]
                                  << " achieved=" << achieved << " expected=" << expected << '\n';
                    require(std::abs(achieved-expected)<=1e-11,
                            "Actual coupled rigid-body angular acceleration must follow the rate-servo demand for every signed axis");
                }
            }
    }
}
void coupled_constant_rate_trim_retains_actual_control_force() {
    // Construct a known Euler equilibrium first, independently of the
    // controller's trim inversion. Axisymmetric inertia and no roll effector
    // permit constant p; nonzero q/r require gyroscopic pitch/yaw torques.
    // Direct fin forces remain nonzero, so compensating only the moment servo
    // while omitting the trim reference cannot preserve this equilibrium.
    for (const double pSign : {-1.,1.}) for (const double qSign : {-1.,1.})
        for (const double rSign : {-1.,1.}) for (const bool measured : {false,true}) {
            auto in=flight();
            in.inertiaKgM2={2,100,100};
            in.rollMomentPerRad=0;
            in.rollDamping=0;
            in.limitsRad[0]=0;
            in.ratesRadS={pSign*.3,qSign*.02,rSign*.015};
            in.alphaRad=qSign*.1;
            in.betaRad=-rSign*.08;
            const auto& w=in.ratesRadS;
            const double pitchTorque=(in.inertiaKgM2[0]-in.inertiaKgM2[2])*w[0]*w[2];
            const double yawTorque=(in.inertiaKgM2[1]-in.inertiaKgM2[0])*w[0]*w[1];
            const double elevator=(pitchTorque-in.pitch.momentAngle*in.alphaRad-in.pitch.momentRate*w[1])/in.pitch.momentControl;
            const double rudder=(yawTorque-in.yaw.momentAngle*in.betaRad-in.yaw.momentRate*w[2])/in.yaw.momentControl;
            const double fz=in.pitch.forceAngle*in.alphaRad+in.pitch.forceControl*elevator;
            const double fy=in.yaw.forceAngle*in.betaRad+in.yaw.forceControl*rudder;
            in.demandBody={0,fy/in.massKg,fz/in.massKg};
            const double gy=in.speedMs*w[2]-in.demandBody[1];
            const double gz=-in.speedMs*w[1]-in.demandBody[2];
            constexpr double gravity=9.80665;
            require(gy*gy+gz*gz<gravity*gravity,
                    "Constant-rate fixture gravity must retain a physically possible direction");
            in.gravityBody={std::sqrt(gravity*gravity-gy*gy-gz*gz),gy,gz};
            if (measured)
                in.measured=AeroFeedback{{0,fy,fz},{0,pitchTorque,yawTorque},{0,elevator,rudder}};
            const auto out=finCommands(in);
            require(out.available && out.aileron==0 && std::abs(out.elevator)<1 && std::abs(out.rudder)<1,
                    "Known constant-rate trim must retain unsaturated two-axis authority");
            require(std::abs(out.elevator*in.limitsRad[1]-elevator)<=1e-12 &&
                        std::abs(out.rudder*in.limitsRad[2]-rudder)<=1e-12,
                    "Gyroscopic constant-rate trim must retain the actual fins and their direct forces");
        }
}
void overflowing_inertial_coupling_is_unavailable_and_neutral() {
    // Every supplied scalar and the decoupled rate-servo arithmetic remain
    // finite, but products of two rates overflow the required Euler torque.
    // Saturating an invalid torque would conceal a missing refusal boundary.
    for (std::size_t zeroAxis=0;zeroAxis<3;++zeroAxis) {
        auto in=flight();
        in.inertiaKgM2={1,100,110};
        in.ratesRadS.fill(1e200);
        in.ratesRadS[zeroAxis]=0;
        const auto out=finCommands(in);
        require(!out.available && out.aileron==0 && out.elevator==0 && out.rudder==0,
                "Overflowing required inertial-coupling torque must fail closed with neutral surfaces");
    }
}
}
int main() {
    unsigned passed = 0;
    for (const auto test : {asymmetric_native_stop_normalization_preserves_requested_radians,
             configured_native_actuator_mismatch_is_unavailable, configured_control_response_and_refusals_are_executed,
             overflowing_inertial_coupling_is_unavailable_and_neutral,
             level_trim_uses_specific_force_and_gravity, signed_tail_and_canard_authority,
             demands_saturate_without_unlimited_authority, no_authority_or_invalid_plant_is_neutral,
             saturated_request_has_no_windup_history, finite_lag_synthetic_plant_tracks_a_step,
             two_axis_canard_does_not_require_roll_authority, nonlinear_native_loads_do_not_become_hidden_tracking_bias,
             aligned_load_feedback_preserves_linear_solution_and_measured_positions,
             coupled_constant_rate_trim_retains_actual_control_force,
             coupled_rigid_body_rates_follow_the_requested_servo_response}) {
        try { test(); ++passed; }
        catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    }
    std::cout << passed << " missile autopilot cases passed\n";
}
