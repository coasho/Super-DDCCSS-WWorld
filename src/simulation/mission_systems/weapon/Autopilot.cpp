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
double reference(const ControlAxis& a, double force, double pathRate) {
    return (force + a.forceControl * a.momentRate * pathRate / a.momentControl) / forceSlope(a);
}
double surface(const ControlAxis& a, double inertia, double angle, double rate, double rateDemand,
               double rateTime, double limit) {
    const double delta = (inertia * (rateDemand - rate) / rateTime -
                          a.momentAngle * angle - a.momentRate * rate) / a.momentControl;
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
    // Demand is specific force including gravity compensation. Path curvature
    // instead uses net acceleration. UVWdot is not an accelerometer signal.
    const double qPath = -(in.demandBody[2] + in.gravityBody[2]) / in.speedMs;
    const double rPath = (in.demandBody[1] + in.gravityBody[1]) / in.speedMs;
    double alpha = reference(in.pitch, in.massKg * in.demandBody[2], qPath);
    double beta = reference(in.yaw, in.massKg * in.demandBody[1], rPath);
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
    out.elevator = surface(in.pitch, in.inertiaKgM2[1], in.alphaRad, in.ratesRadS[1], qDemand, rateTime, in.limitsRad[1]);
    out.rudder = surface(in.yaw, in.inertiaKgM2[2], in.betaRad, in.ratesRadS[2], rDemand, rateTime, in.limitsRad[2]);
    // Some authored two-axis canards have no roll effector. Their valid pitch
    // and yaw channels still operate; passive roll damping stays in the plant.
    if (in.limitsRad[0] > 0 && std::abs(in.rollMomentPerRad) > 1e-9)
        out.aileron = (-in.inertiaKgM2[0] * in.ratesRadS[0] / rateTime - in.rollDamping * in.ratesRadS[0]) /
                       (in.rollMomentPerRad * in.limitsRad[0]);
    if (!std::isfinite(out.elevator) || !std::isfinite(out.rudder) || !std::isfinite(out.aileron)) return {};
    out.elevator = std::clamp(out.elevator, -1., 1.);
    out.rudder = std::clamp(out.rudder, -1., 1.);
    out.aileron = std::clamp(out.aileron, -1., 1.);
    out.available = true;
    return out;
}
}
