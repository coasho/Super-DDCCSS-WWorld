#include "simulation/mission_systems/weapon/Autopilot.h"

#include <algorithm>
#include <cmath>

namespace fsim::ms::weapon {
namespace {
bool valid(const ControlAxis& a) {
    return std::isfinite(a.forceAngle) && std::isfinite(a.forceControl) &&
        std::isfinite(a.momentAngle) && std::isfinite(a.momentControl) && std::isfinite(a.momentRate) &&
        std::abs(a.momentControl) > 1e-9;
}
double forceSlope(const ControlAxis& a) {
    return a.forceAngle - a.forceControl * a.momentAngle / a.momentControl;
}
double reference(const ControlAxis& a, double force, double pathRate, double forceResidual, double momentResidual) {
    return (force - forceResidual + a.forceControl * (a.momentRate * pathRate + momentResidual) / a.momentControl) / forceSlope(a);
}
double surface(const ControlAxis& a, double inertia, double angle, double rate, double rateDemand,
               double rateTime, double limit, double momentResidual, double couplingMoment) {
    const double delta = (inertia * (rateDemand - rate) / rateTime + couplingMoment -
                          a.momentAngle * angle - a.momentRate * rate - momentResidual) / a.momentControl;
    return delta / limit;
}
}
FinCommands finCommands(const AutopilotInput& in) noexcept {
    if (!std::isfinite(in.massKg) || in.massKg <= 0 || !std::isfinite(in.speedMs) || in.speedMs < 1 ||
        !std::isfinite(in.actuatorLagS) || in.actuatorLagS <= 0 ||
        !std::isfinite(in.alphaRad) || !std::isfinite(in.betaRad) ||
        std::cos(in.alphaRad) * std::cos(in.betaRad) <= 0 || !valid(in.pitch) || !valid(in.yaw) ||
        !std::isfinite(in.rollMomentPerRad) ||
        !std::isfinite(in.rollDamping)) return {};
    for (std::size_t k = 0; k < 3; ++k)
        if (!std::isfinite(in.demandBody[k]) || !std::isfinite(in.gravityBody[k]) ||
            !std::isfinite(in.ratesRadS[k]) || !std::isfinite(in.inertiaKgM2[k]) || in.inertiaKgM2[k] <= 0 ||
            !std::isfinite(in.limitsRad[k]) || in.limitsRad[k] < 0 || (k != 0 && in.limitsRad[k] == 0)) return {};
    for (const auto* a : {&in.pitch, &in.yaw}) {
        const double effective = forceSlope(*a);
        if (!std::isfinite(effective) || std::abs(effective) < 1e-6 * std::max(std::abs(a->forceAngle), 1.)) return {};
    }
    // Euler's diagonal-inertia rigid body requires M = J*omega_dot +
    // omega cross (J*omega). Aerodynamic-load feedback does not contain this
    // inertial coupling, including when its aerodynamic measurement is exact.
    const auto& w=in.ratesRadS;
    const auto& inertia=in.inertiaKgM2;
    const std::array<double,3> couplingMoment={
        (inertia[2]-inertia[1])*w[1]*w[2],
        (inertia[0]-inertia[2])*w[0]*w[2],
        (inertia[1]-inertia[0])*w[0]*w[1]};
    for (const double moment : couplingMoment)
        if (!std::isfinite(moment)) return {};
    // Account for loads already present in the native model but absent from
    // its small-incidence derivatives (crossflow, finite fin geometry, etc.).
    // This is a local additive residual, not a replacement force or a fitted
    // effectiveness derivative. Use measured actual fins from the SAME frame.
    double pitchForce=0,yawForce=0,pitchMoment=0,yawMoment=0,rollMoment=0;
    if (in.measured) {
        const auto& m=*in.measured;
        for (std::size_t k=0;k<3;++k)
            if (!std::isfinite(m.forceBodyN[k]) || !std::isfinite(m.momentBodyNm[k]) || !std::isfinite(m.controlRad[k])) return {};
        pitchForce=m.forceBodyN[2]-in.pitch.forceAngle*in.alphaRad-in.pitch.forceControl*m.controlRad[1];
        yawForce=m.forceBodyN[1]-in.yaw.forceAngle*in.betaRad-in.yaw.forceControl*m.controlRad[2];
        pitchMoment=m.momentBodyNm[1]-in.pitch.momentAngle*in.alphaRad-in.pitch.momentRate*in.ratesRadS[1]-in.pitch.momentControl*m.controlRad[1];
        yawMoment=m.momentBodyNm[2]-in.yaw.momentAngle*in.betaRad-in.yaw.momentRate*in.ratesRadS[2]-in.yaw.momentControl*m.controlRad[2];
        rollMoment=m.momentBodyNm[0]-in.rollDamping*in.ratesRadS[0]-in.rollMomentPerRad*m.controlRad[0];
        for (double residual : {pitchForce,yawForce,pitchMoment,yawMoment,rollMoment})
            if (!std::isfinite(residual)) return {};
    }
    // Demand is specific force including gravity compensation. Path curvature
    // instead uses net acceleration. UVWdot is not an accelerometer signal.
    const double qPath = -(in.demandBody[2] + in.gravityBody[2]) / in.speedMs;
    const double rPath = (in.demandBody[1] + in.gravityBody[1]) / in.speedMs;
    // Constant-rate trim also needs the coupling torque. Its balancing fin
    // deflection contributes direct force, so compensate before solving trim.
    double alpha = reference(in.pitch, in.massKg * in.demandBody[2], qPath,pitchForce,pitchMoment-couplingMoment[1]);
    double beta = reference(in.yaw, in.massKg * in.demandBody[1], rPath,yawForce,yawMoment-couplingMoment[2]);
    if (!std::isfinite(alpha) || !std::isfinite(beta)) return {};
    // Assumed reduced-order control envelope: combined incidence <=15 degrees.
    constexpr double maxIncidence = .2617993877991494;
    const double incidence = std::hypot(alpha, beta);
    if (!std::isfinite(incidence)) return {};
    if (incidence > maxIncidence) alpha *= maxIncidence / incidence, beta *= maxIncidence / incidence;
    // Estimated bandwidth, explicitly slower than the finite actuator. The
    // nominal 40 ms actuator gives 120 ms rate and 400 ms incidence response.
    const double rateTime = std::max(.12, 3 * in.actuatorLagS);
    const double incidenceTime = std::max(.4, 3.333333333333333 * rateTime);
    const double qDemand = qPath + (alpha - in.alphaRad) / incidenceTime;
    const double rDemand = rPath - (beta - in.betaRad) / incidenceTime;
    FinCommands out;
    out.elevator = surface(in.pitch, in.inertiaKgM2[1], in.alphaRad, in.ratesRadS[1], qDemand, rateTime, in.limitsRad[1],pitchMoment,couplingMoment[1]);
    out.rudder = surface(in.yaw, in.inertiaKgM2[2], in.betaRad, in.ratesRadS[2], rDemand, rateTime, in.limitsRad[2],yawMoment,couplingMoment[2]);
    // Some authored two-axis canards have no roll effector. Their valid pitch
    // and yaw channels still operate; passive roll damping stays in the plant.
    if (in.limitsRad[0] > 0 && std::abs(in.rollMomentPerRad) > 1e-9)
        out.aileron = (-in.inertiaKgM2[0] * in.ratesRadS[0] / rateTime + couplingMoment[0] - in.rollDamping * in.ratesRadS[0] - rollMoment) /
                       (in.rollMomentPerRad * in.limitsRad[0]);
    if (!std::isfinite(out.elevator) || !std::isfinite(out.rudder) || !std::isfinite(out.aileron)) return {};
    out.elevator = std::clamp(out.elevator, -1., 1.);
    out.rudder = std::clamp(out.rudder, -1., 1.);
    out.aileron = std::clamp(out.aileron, -1., 1.);
    out.available = true;
    return out;
}
}
